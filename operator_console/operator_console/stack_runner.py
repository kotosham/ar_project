"""Супервизор дочернего `ros2 launch`: старт, останов, кольцевой журнал.

ЧЕГО ЗДЕСЬ НЕТ И ПОЧЕМУ
=======================
Здесь НЕТ определения фазы подъёма по строкам stdout. Соблазн был: набор из семи
регулярок («Gazebo Sim», «Managed nodes are active», «YOLOE» ...) выглядит
дешёвым индикатором прогресса. Но фаза подъёма уже известна ТОЧНО и из надёжного
источника — `/robot_health` даёт состояние каждого компонента, `/heartbeat` даёт
живость детектора и оркестратора, а lifecycle-запрос даёт реальное состояние
Nav2. Регулярки по чужому логу дублировали бы это, отставали бы от него и
ломались бы при первой смене формулировки в апстриме Nav2 или RTAB-Map — то есть
показывали бы «поднялось» там, где ничего не поднялось. Поэтому `phase` здесь
описывает ровно то, что этот модуль действительно знает: состояние ПРОЦЕССА.

Что осталось: запуск, честный останов группы процессов, кольцевой журнал и
необязательный tee в файл. Tee — единственная строка в `_pump`, но она спасает
разбор упавшего прогона: журнал живёт только в памяти консоли и умирает вместе с
процессом, а падение стека обычно и заканчивается перезапуском консоли.

`popen` и `clock` инъектируемы — именно это делает класс тестируемым без ROS.
"""
import os
import signal
import subprocess
import threading
import time
from collections import deque

# Единый корневой launch. Промежуточный слой был чистым passthrough и слит с
# корнем, поэтому файл ровно один (второй, edge_layer.launch.py, поднимается
# изнутри него, а не отсюда).
LAUNCH_PACKAGE = 'ar_project'
LAUNCH_FILE = 'mission_bringup.launch.py'

PHASE_IDLE = 'idle'          # процесса не было
PHASE_STARTING = 'starting'  # запущен, но ещё ни строки в stdout
PHASE_RUNNING = 'running'    # процесс жив и говорит
PHASE_EXITED = 'exited'      # завершился сам, код 0
PHASE_FAILED = 'failed'      # завершился с ненулевым кодом или был убит
PHASE_STOPPED = 'stopped'    # остановлен оператором

# Строки, по которым видно ЖЁСТКИЙ отказ. Это не определение фазы: здесь нет
# попытки угадать прогресс — только зафиксировать первый признак того, что
# запуск не состоялся, чтобы не заставлять оператора листать журнал.
FAIL_MARKERS = (
    'process has died',
    'failed to load',
    'no such file or directory',
    'command not found',
    'package not found',
    'Traceback (most recent call last)',
)


def looks_failed(line):
    """Похожа ли строка журнала на признак несостоявшегося запуска."""
    lowered = (line or '').lower()
    return any(marker.lower() in lowered for marker in FAIL_MARKERS)


def build_launch_argv(*, mode, planner, layer, world_file, gui, rviz,
                      dashboard_port, venv_python, extra=None,
                      launch_file=LAUNCH_FILE):
    """Аргументы дочернего процесса. Чистая функция — сравнивается списком в тесте.

    `venv_python` прогоняется через expanduser здесь: аргумент уходит в argv, где
    никакой оболочки, которая раскроет '~', уже нет, и launch получил бы путь
    вида './~/.venvs/...' (ровно на это наступил vlm_sim_bringup.launch.py:70).
    """
    argv = [
        'ros2', 'launch', LAUNCH_PACKAGE, launch_file,
        'mode:=%s' % mode,
        'planner:=%s' % planner,
        'layer:=%s' % layer,
        'world:=%s' % (world_file or ''),
        'gui:=%s' % ('true' if gui else 'false'),
        'rviz:=%s' % ('true' if rviz else 'false'),
        'dashboard_port:=%d' % int(dashboard_port),
        'venv_python:=%s' % os.path.expanduser(venv_python or ''),
    ]
    for key in sorted(extra or {}):
        argv.append('%s:=%s' % (key, extra[key]))
    return argv


class StackRunner:
    def __init__(self, log_capacity=4000, popen=subprocess.Popen,
                 clock=time.monotonic):
        self._lines = deque(maxlen=log_capacity)
        self._seq = 0
        self._proc = None
        self._phase = PHASE_IDLE
        self._started_at = None      # стенное время, для показа человеку
        self._started_mono = None    # монотонное, для uptime
        self._last_error = None
        self._exit_code = None
        self._tee = None
        self._tee_path = None
        self._argv = []
        self._lock = threading.RLock()
        self._popen = popen
        self._clock = clock

    # -- запуск -------------------------------------------------------------

    def start(self, argv, env=None, cwd=None, log_path=None):
        with self._lock:
            if self.is_running():
                raise RuntimeError('стек уже запущен (pid %s)'
                                   % getattr(self._proc, 'pid', '?'))
            self._lines.clear()
            self._seq = 0
            self._last_error = None
            self._exit_code = None
            self._argv = list(argv)

            kwargs = {
                'stdout': subprocess.PIPE,
                'stderr': subprocess.STDOUT,
                'text': True,
                'bufsize': 1,
                'env': dict(os.environ, **(env or {})),
                'cwd': cwd,
            }
            if os.name == 'nt':
                # На Windows start_new_session не поддерживается; аналог, который
                # даёт возможность послать CTRL_BREAK всей группе, — это флаг.
                kwargs['creationflags'] = getattr(subprocess,
                                                  'CREATE_NEW_PROCESS_GROUP', 0)
            else:
                # ОБЯЗАТЕЛЬНО: ros2 launch порождает дерево процессов, и убить
                # его целиком можно только сигналом по группе. Без своей сессии
                # сигнал ушёл бы и самой консоли.
                kwargs['start_new_session'] = True

            self._tee = self._open_tee(log_path)
            self._proc = self._popen(argv, **kwargs)
            self._phase = PHASE_STARTING
            self._started_at = time.time()
            self._started_mono = self._clock()

            thread = threading.Thread(target=self._pump, args=(self._proc,),
                                      daemon=True, name='stack-log-pump')
            thread.start()
            return self.status()

    def _open_tee(self, log_path):
        if not log_path:
            return None
        path = os.path.expanduser(log_path)
        try:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            handle = open(path, 'a', encoding='utf-8', errors='replace')
        except OSError:
            # Невозможность писать журнал на диск не повод не запускать стек.
            self._tee_path = None
            return None
        self._tee_path = path
        return handle

    def _pump(self, proc):
        """Построчное чтение stdout дочернего процесса до его завершения."""
        stream = getattr(proc, 'stdout', None)
        if stream is not None:
            try:
                for raw in iter(stream.readline, ''):
                    if raw == '':
                        break
                    text = raw.rstrip('\r\n')
                    with self._lock:
                        self._seq += 1
                        self._lines.append({'n': self._seq, 't': time.time(),
                                            'stream': 'out', 'text': text})
                        if self._phase == PHASE_STARTING:
                            self._phase = PHASE_RUNNING
                        if self._last_error is None and looks_failed(text):
                            self._last_error = text[:500]
                        tee = self._tee
                    if tee is not None:
                        # Одна строка, ради которой tee вообще есть: журнал в
                        # памяти умирает вместе с консолью, а разбирать надо
                        # именно упавший прогон.
                        try:
                            tee.write(text + '\n')
                            tee.flush()
                        except (OSError, ValueError):
                            # ValueError = файл закрыт из stop() параллельно;
                            # потеря хвоста журнала не повод ронять поток чтения.
                            pass
            except (ValueError, OSError):
                # Поток закрыт из stop() — штатное завершение чтения.
                pass

        code = None
        try:
            code = proc.wait()
        except Exception:
            code = getattr(proc, 'returncode', None)
        with self._lock:
            self._exit_code = code
            if self._phase != PHASE_STOPPED:
                self._phase = PHASE_EXITED if code == 0 else PHASE_FAILED
            if code not in (0, None) and self._last_error is None:
                self._last_error = ('процесс завершился с кодом %s' % code)
            self._close_tee()

    # -- останов ------------------------------------------------------------

    def stop(self, grace_s=20.0):
        """SIGINT -> SIGTERM -> SIGKILL по ГРУППЕ процессов.

        SIGINT даётся полный grace_s, а не пара секунд, намеренно. `ros2 launch`
        по SIGINT корректно гасит дерево узлов, и они успевают освободить
        сегменты разделяемой памяти. SIGKILL этого шанса не даёт: убитый жёстко
        узел оставляет в /dev/shm сегменты Fast DDS, из-за которых СЛЕДУЮЩИЙ
        запуск может не подняться вообще — этот отказ уже наблюдался на пакетных
        прогонах бенчмарка и выглядит как «стек молча не стартует».
        """
        with self._lock:
            proc = self._proc
            if proc is None or not self.is_running():
                self._phase = PHASE_STOPPED if proc is not None else PHASE_IDLE
                return self.status()
            self._phase = PHASE_STOPPED

        self._signal(proc, 'int')
        if self._wait(proc, grace_s) is None:
            self._signal(proc, 'term')
            if self._wait(proc, 5.0) is None:
                self._signal(proc, 'kill')
                self._wait(proc, 5.0)
        with self._lock:
            self._exit_code = getattr(proc, 'returncode', None)
            self._close_tee()
        return self.status()

    def _signal(self, proc, kind):
        pid = getattr(proc, 'pid', None)
        if pid is None:
            return
        try:
            if os.name == 'nt':
                if kind == 'int':
                    proc.send_signal(getattr(signal, 'CTRL_BREAK_EVENT',
                                             signal.SIGTERM))
                elif kind == 'term':
                    proc.terminate()
                else:
                    proc.kill()
                return
            group = os.getpgid(pid)
            os.killpg(group, {'int': signal.SIGINT,
                              'term': signal.SIGTERM}.get(kind, signal.SIGKILL))
        except (OSError, ProcessLookupError, AttributeError, ValueError):
            # Процесс уже мёртв либо группы нет — это не ошибка останова.
            pass

    @staticmethod
    def _wait(proc, timeout_s):
        try:
            return proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            return None
        except Exception:
            return getattr(proc, 'returncode', None)

    def _close_tee(self):
        if self._tee is not None:
            try:
                self._tee.close()
            except OSError:
                pass
            self._tee = None

    # -- состояние ----------------------------------------------------------

    def is_running(self):
        proc = self._proc
        if proc is None:
            return False
        try:
            return proc.poll() is None
        except Exception:
            return False

    def status(self):
        with self._lock:
            running = self.is_running()
            uptime = None
            if self._started_mono is not None:
                uptime = round(self._clock() - self._started_mono, 1)
            return {
                'running': running,
                'pid': getattr(self._proc, 'pid', None) if self._proc else None,
                'phase': self._phase,
                'phase_ru': PHASE_RU.get(self._phase, self._phase),
                'started_at': self._started_at,
                'uptime_s': uptime,
                'exit_code': self._exit_code,
                'last_error': self._last_error,
                'log_lines': len(self._lines),
                'log_path': self._tee_path,
                'argv': list(self._argv),
            }

    def log(self, since=0, limit=500):
        with self._lock:
            lines = [item for item in self._lines if item['n'] > since]
            total = self._seq
        chunk = lines[:max(0, int(limit))]
        next_n = chunk[-1]['n'] if chunk else since
        return {'lines': chunk, 'next': next_n, 'total': total}

    def close(self):
        """Вызывается при завершении узла: оставлять осиротевший `ros2 launch`
        нельзя — он продолжит держать Gazebo и порты."""
        try:
            if self.is_running():
                self.stop(grace_s=10.0)
        finally:
            with self._lock:
                self._close_tee()


PHASE_RU = {
    PHASE_IDLE: 'не запускался',
    PHASE_STARTING: 'запускается',
    PHASE_RUNNING: 'работает',
    PHASE_EXITED: 'завершён',
    PHASE_FAILED: 'аварийно завершён',
    PHASE_STOPPED: 'остановлен оператором',
}
