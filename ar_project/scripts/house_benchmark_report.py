#!/usr/bin/env python3
"""house_benchmark_report.py - offline aggregation and diagnostic reporting for the house benchmark.

This is the metrics layer of the house benchmark: `house_scenario_runner.py` produces one JSON
episode record per (scenario, perturbation, planner, seed) run, and this script turns a directory
of those records into per-episode tables, aggregates, paired planner deltas, perturbation
sensitivity, failure attribution and a Markdown report.

Dependency policy: standard library only, plus PyYAML *if it happens to be installed* (used solely
to read the perturbation overlays when --perturbation-dir is given, so that "does this perturbation
touch the camera?" can be answered from the config instead of from the id string). No ROS, no
pandas, no numpy. The tool must run on a laptop with a plain python3.

-------------------------------------------------------------------------------------------------
THE PAIRED PROTOCOL (why this file exists)
-------------------------------------------------------------------------------------------------
The headline question of the benchmark is not "how good is the VLM planner in absolute terms" -
that number is dominated by the house, the robot and the detector, none of which are under test.
The question is "what does swapping the planner change, holding everything else fixed".

So the runner is expected to execute every (scenario_id, perturbation_id, seed) triple twice: once
with the baseline planner (default `flat_mock`) and once with the treatment planner (default
`vlm`). Because the seed fixes the spawn jitter, the start pose noise and any stochastic policy
choices, the two runs of a triple are as close to a controlled A/B as a simulator allows. We
therefore compare them *within* the triple:

    delta_progress = ordered_progress(treatment) - ordered_progress(baseline)
    delta_success  = success(treatment)          - success(baseline)

and only then average across triples. Averaging the two planners separately and subtracting the
means would be legal but far noisier: scenario/seed difficulty is a huge variance term and pairing
cancels it exactly. Episodes whose partner is missing are NOT silently dropped into the mean - they
are counted and listed as `unpaired`, and `--strict` turns them into a non-zero exit so that a
half-finished sweep can never masquerade as a result.

-------------------------------------------------------------------------------------------------
WHY THE DEAD-BAND
-------------------------------------------------------------------------------------------------
`ordered_progress` is not a continuous quantity. It is (completed prefix of the ordered subgoal
list) / n_subgoals, so for a 4-subgoal scenario it can only ever be 0, 0.25, 0.5, 0.75 or 1.0. Its
quantum is exactly 1 / n_subgoals. A non-zero delta smaller than one subgoal cannot be produced by
"the treatment planner got further"; it can only come from mixing scenarios with different
n_subgoals, from partial-credit variants of the metric, or from float noise. Calling such a delta a
win would be reading meaning into the last significant digit.

We therefore classify each pair with a dead-band of epsilon = 1 / n_subgoals:

    delta_progress >= +epsilon  -> benefit   (the treatment reached at least one more subgoal)
    delta_progress <= -epsilon  -> harm      (the treatment lost at least one subgoal)
    |delta_progress| <  epsilon -> neutral   (below the resolution of the metric)

benefit_rate / harm_rate / neutral_rate are the fractions of pairs in each class. The dead-band is
per-pair, because n_subgoals is per-scenario; the report prints the epsilon it used for every row so
the reader can check it.

-------------------------------------------------------------------------------------------------
ROBUSTNESS CONTRACT
-------------------------------------------------------------------------------------------------
* Files that are not valid JSON, are not objects, or carry no `schema_version` are skipped and
  listed in the data-quality section - never fatal, never silently ignored.
* Every field is read through `_get()`, which substitutes a documented default and counts the miss,
  so a runner that gains or loses a field does not break the report; the miss count shows up in the
  data-quality table instead.
* Everything is sorted before it is written, so two runs over the same directory produce
  byte-identical files and reports diff cleanly.
"""

import argparse
import csv
import glob
import json
import math
import os
import re
import shutil
import sys
import tempfile
from datetime import datetime, timezone

# --------------------------------------------------------------------------------------------
# Constants describing the fixture under test. These are prose only - the numbers come from the
# authoritative world file, they are repeated here so that a report is self-contained when it is
# mailed around without the repo.
# --------------------------------------------------------------------------------------------

HOUSE_DESCRIPTION = [
    "Дом 15 x 10 м (пол x in [-7.5, 7.5], y in [-5.0, 5.0]), высота стен 2.2 м, без потолка.",
    "Коридор - полоса y in [-1.1, +1.1] на всю длину дома: непрерывная линия видимости 14.8 м.",
    "Комнаты: bedroom, bathroom, kitchen (север, y > +1.1); storage, living (юг, y < -1.1).",
    "Проёмы: bedroom/storage x in [-4.60, -3.50], bathroom x in [-0.55, +0.55],",
    "        kitchen x in [+3.60, +4.70], living x in [+2.40, +3.50].",
    "Свет - только точечные источники уровня world (light_hallway_*, light_bedroom, ...),",
    "их можно гасить на лету, поэтому 'lights_off' - настоящее возмущение, а не постобработка.",
    "Робот: дифференциальный привод, footprint ~0.40 x 0.35 м, КАМЕРА НА ВЫСОТЕ 0.13 м,",
    "HFOV 62.4 deg. Depth/scan обрезаны на 8 м, RGB видит до 30 м - отсюда 'depth gap'.",
    "Разрешение НЕ фиксировано: 320x240 в обычных bring-up'ах, 640x480 по умолчанию в",
    "house_sim.launch.py (нужно для OCR-сценариев). Сравнивать можно только эпизоды,",
    "снятые при одном разрешении - оно меняет и детектор, и то, что видит VLM.",
]

# Diagnostic axes of the suite. The tag on the left is a REAL tag from the `diagnoses:` list of at
# least one shipped scenario YAML (the runner now carries those tags into the episode record, and
# the report prints them per scenario), so this block and the data speak the same vocabulary.
# Perturbation-driven stress is deliberately NOT listed here: it is not a property of a scenario,
# it is the perturbation_id axis, and it gets its own sensitivity table.
DIAGNOSTIC_AXES = [
    ("depth_gap",
     "Цель дальше 8 м: depth/scan молчат, цель видна только в RGB. Проверяет, умеет ли планировщик "
     "двигаться к тому, что видно, но не измеряется. (s1)"),
    ("multi_step_decomposition",
     "Один большой подход надо разбить на несколько шагов вперёд. Проверяет декомпозицию, "
     "а не жадный рывок. (s1)"),
    ("viewpoint_change",
     "Цель закрыта occluder_screen: с текущей точки grounding невозможен. Проверяет смену ракурса "
     "вместо кружения вокруг последней детекции. (s2)"),
    ("active_search",
     "Цели нет в стартовом кадре, а впереди стоит приманка. Проверяет поворот-до-поездки и "
     "устойчивость к жадному выбору видимого. (s3)"),
    ("ocr",
     "В кадре есть читаемый текст или стрелка-указатель. Проверяет, превращает ли планировщик "
     "прочитанное в цель навигации. (s4, s5)"),
    ("distractor_resistance",
     "Среди указателей есть верный, но нерелевантный цели. Проверяет целеобусловленное чтение, "
     "а не следование любой подсказке. (s5)"),
    ("semantic_inference",
     "Миссия названа помещением; детектор возвращает только объекты. Проверяет вывод "
     "'унитаз -> это ванная'. (s6)"),
    ("room_adjacency",
     "Цель за глухой стеной соседней комнаты. Проверяет рассуждение о топологии и выход из "
     "комнаты до начала поиска. (s7)"),
]

# --------------------------------------------------------------------------------------------
# Schema. Keys are the fields house_scenario_runner.py writes; values are the default substituted
# when the field is absent (and the absence is counted for the data-quality section).
# Fields marked OPTIONAL are not part of the documented record schema - the tool uses them when the
# runner happens to provide them and falls back to inference when it does not.
# --------------------------------------------------------------------------------------------

FIELD_DEFAULTS = {
    "schema_version": 0,
    "episode_key": "",
    "scenario_id": "",
    "perturbation_id": "",
    "planner_label": "",
    "seed": -1,
    "mission": "",
    "wall_duration_s": 0.0,
    "sim_duration_s": 0.0,
    "outcome": "",
    "outcome_reason": "",
    "n_subgoals": 0,
    "ordered_progress": 0.0,
    "unordered_progress": 0.0,
    "progress_auc": 0.0,
    "first_failure_subgoal": "",
    "subgoal_times": {},
    "path_length_m": 0.0,
    "max_path_m": 0.0,
    "timeout_s": 0.0,
    "collided": False,
    "vlm_steps": 0,
    "action_histogram": {},
    "detect_all_calls": 0,
    "plan_failures": 0,
    "degraded_events": 0,
    "notes_events": 0,
    "pose_source": "",
    # The runner writes `spawned` as a MAPPING name -> {model, x, y, z, yaw}, not a list.
    # Defaulting it to [] used to send it through _as_list(), which returns [] for a dict,
    # so spawned_n was silently 0 on every episode that actually spawned props.
    "spawned": {},
    "lights_off": [],
}
# NOTE: `diagnoses` deliberately lives in OPTIONAL_FIELDS below, not here. The runner writes it
# (scenario tags: ocr, depth_gap, room_adjacency, ...) so results can be grouped by WHAT is being
# diagnosed rather than by scenario id, but records produced before that existed legitimately lack
# it -- counting it as a missing required field would flag every such record as damaged.

OPTIONAL_FIELDS = {
    # If the runner records which labels ever came back from detect_all, grounding attribution
    # becomes exact instead of inferred. Absence is normal and is not reported as a problem.
    "seen_labels": None,
    "target_label": None,
    "scenario_title": None,
    "diagnoses": None,
    "camera_perturbed": None,
    "max_vlm_steps": None,
}

OUTCOME_SUCCESS = "success"
OUTCOME_TIMEOUT = "timeout"
OUTCOME_SETUP_FAILED = "setup_failed"
NO_PERTURBATION = "p_none"

# Detector vocabulary of the spawnable props, used to pull a target label out of the free-form
# mission string when the runner does not record one explicitly. Longest first so that
# "sports ball" wins over "ball".
TARGET_VOCAB = [
    "sports ball", "bottle", "towel", "toilet", "chair", "cup", "ball", "sink", "bed", "sofa",
    "fridge",
]

# Missions that name a ROOM, not a detectable object class. s6/s7 publish "bathroom" on
# /vlm_mission on purpose: the detector will never return that label, the planner has to infer the
# room from what it does see (a toilet). Attributing those episodes to `grounding` -- "the target
# was never detected" -- would be a guaranteed false positive on exactly the two scenarios whose
# whole point is that the target is not detectable. They fall through to the later buckets.
ROOM_LEVEL_MISSIONS = frozenset((
    "bathroom", "kitchen", "bedroom", "living", "living room", "storage", "hallway",
))

# Perturbation ids that contain any of these tokens are assumed to degrade the camera. Used only
# when neither an explicit record field nor a parsed overlay YAML is available.
# "dirty"/"lens"/"hard" are in the list because the shipped overlay set names its most
# camera-destroying case `p_dirty_lens` and its combined case `p_hard`: without those tokens the
# `perception` bucket could never fire for the very perturbation it exists to catch.
CAMERA_PERTURBATION_TOKENS = (
    "cam", "smudge", "blur", "dark", "noise", "dropout", "motion", "light", "dim", "glare",
    "dirty", "lens", "grease", "fog", "hard",
)

SAW_SUBGOAL_RE = re.compile(r"(^|[_\-])(saw|see|seen|sees|detect|detected|spot|spotted|observe)")
TURN_ACTION_RE = re.compile(r"turn|rotate|spin|yaw", re.IGNORECASE)

# --------------------------------------------------------------------------------------------
# Tunable thresholds. Every attribution rule reads its numbers from here and the report prints this
# dict verbatim in an appendix, so a reader can re-derive any bucket by hand.
# --------------------------------------------------------------------------------------------

THRESHOLDS = {
    # share of TURN-like actions above which the action histogram is called "churn"
    "churn_turn_share": 0.60,
    # share of ONE single action above which the policy is called "stuck repeating"
    "churn_dominant_share": 0.90,
    # a histogram with fewer steps than this is too short to call anything churn
    "churn_min_steps": 8,
    # progress counts as "still rising at the end" if the last subgoal completed after this
    # fraction of the episode duration
    "progress_tail_fraction": 0.75,
    # path length above which "the robot kept driving while progress stood still" is meaningful
    "execution_min_path_m": 3.0,
    # nav/skill failures at or above this count dominate the episode
    "execution_min_plan_failures": 3,
    "execution_min_degraded": 5,
    # ordered_progress drop versus the same scenario/seed under p_none that we are willing to blame
    # on the camera perturbation
    "perception_min_drop": 0.25,
    # ordered_progress at or above this counts as "essentially complete" and never stalled
    "progress_complete": 0.999,
}


# --------------------------------------------------------------------------------------------
# Attribution predicates. Defined before ATTRIBUTION_RULES because the dict stores the functions.
# Each returns None (rule does not fire) or a short human-readable reason string.
# `ep` is a normalised episode dict, `ctx` carries the cross-episode references.
# --------------------------------------------------------------------------------------------

def _rule_setup(ep, ctx):
    if ep["outcome"] == OUTCOME_SETUP_FAILED:
        return "outcome=setup_failed"
    return None


def _rule_catastrophic(ep, ctx):
    if ep["collided"]:
        return "collided=true"
    return None


def _rule_perception(ep, ctx):
    if not ep["camera_perturbed"]:
        return None
    ref = ctx.reference_progress(ep)
    if ref is None:
        return None
    drop = ref - ep["ordered_progress"]
    if drop >= THRESHOLDS["perception_min_drop"]:
        return "camera perturbation, ordered_progress %.2f vs %.2f under %s (drop %.2f)" % (
            ep["ordered_progress"], ref, NO_PERTURBATION, drop)
    return None


def _rule_grounding(ep, ctx):
    # A room-level mission has no detectable target by construction (see ROOM_LEVEL_MISSIONS),
    # so "never detected" carries no information and must not consume the episode.
    if str(ep.get("mission", "")).strip().lower() in ROOM_LEVEL_MISSIONS:
        return None
    if ep["saw_target"] is False:
        return "target label '%s' never confirmed; first_failure_subgoal=%s" % (
            ep["target_label"] or "?", ep["first_failure_subgoal"] or "-")
    return None


def _rule_planner(ep, ctx):
    if not _stalled(ep):
        return None
    hist_steps = ep["hist_total"]
    if hist_steps < THRESHOLDS["churn_min_steps"]:
        return None
    if ep["turn_share"] > THRESHOLDS["churn_turn_share"]:
        return "progress stalled, %.0f%% of %d actions are TURN" % (
            100.0 * ep["turn_share"], hist_steps)
    if ep["dominant_share"] >= THRESHOLDS["churn_dominant_share"]:
        return "progress stalled, action '%s' repeated %.0f%% of %d steps" % (
            ep["dominant_action"], 100.0 * ep["dominant_share"], hist_steps)
    return None


def _rule_execution(ep, ctx):
    if _stalled(ep) and ep["path_length_m"] >= THRESHOLDS["execution_min_path_m"]:
        return "progress stalled but path_length grew to %.1f m" % ep["path_length_m"]
    if ep["plan_failures"] >= THRESHOLDS["execution_min_plan_failures"]:
        return "plan_failures=%d dominate the episode" % ep["plan_failures"]
    if ep["degraded_events"] >= THRESHOLDS["execution_min_degraded"]:
        return "degraded_events=%d dominate the episode" % ep["degraded_events"]
    return None


def _rule_timeout(ep, ctx):
    if ep["timed_out"] and ep["progress_rising_at_end"]:
        return "hit timeout_s=%.0f with progress still rising" % ep["timeout_s"]
    if ep["timed_out"]:
        return "hit timeout_s=%.0f" % ep["timeout_s"]
    return None


def _rule_unattributed(ep, ctx):
    # Residual bucket. It exists so the tool can never crash or silently drop an episode; a
    # non-zero count is a defect in the rule table above, not a property of the robot.
    return "no rule matched"


# ORDER MATTERS - first match wins. The order is chosen so that a cause which invalidates all later
# evidence is tested first:
#   1 setup        nothing was ever attempted, so no other metric of this episode means anything
#   2 catastrophic a collision ends the episode physically; whatever else was true is moot
#   3 perception   if the camera was deliberately degraded AND the same scenario/seed did much
#                  better under p_none, the disturbance explains the failure - blaming the
#                  planner or the detector here would be attributing our own perturbation to them
#   4 grounding    the target was never confirmed, so the planner never had anything to plan for
#   5 planner      the target WAS seen, yet the action histogram churns and progress stands still
#   6 execution    the plan looks sane (no churn) but the body does not deliver: distance burned
#                  with no progress, or nav/skill failures dominate
#   7 timeout      nothing pathological, the clock simply ran out (progress was still rising)
#   8 unattributed residual, must stay empty
ATTRIBUTION_ORDER = (
    "setup", "catastrophic", "perception", "grounding", "planner", "execution", "timeout",
    "unattributed",
)

ATTRIBUTION_RULES = {
    "setup": {
        "order": 1,
        "test": _rule_setup,
        "rule": "outcome == 'setup_failed'",
        "why": "Эпизод не стартовал (спавн/сервис gz). Это дефект стенда, не робота - такие "
               "эпизоды нельзя смешивать с провалами планирования.",
    },
    "catastrophic": {
        "order": 2,
        "test": _rule_catastrophic,
        "rule": "collided == true",
        "why": "Столкновение обесценивает остальную телеметрию: прогресс после удара недостоверен.",
    },
    "perception": {
        "order": 3,
        "test": _rule_perception,
        "rule": "camera_perturbed and (ordered_progress(p_none, same scenario+seed+planner) - "
                "ordered_progress) >= perception_min_drop",
        "why": "Мы сами испортили камеру. Если тот же сценарий с тем же seed под p_none шёл "
               "заметно дальше - причина в возмущении, а не в планировщике.",
    },
    "grounding": {
        "order": 4,
        "test": _rule_grounding,
        "rule": "target label never confirmed by detect_all (explicit seen_labels, or "
                "first_failure_subgoal is a saw_* subgoal, or detect_all_calls == 0 with zero "
                "progress); NOT applied to room-level missions",
        "why": "Планировщику нечего было планировать: цель не была найдена ни разу. Сценарии с "
               "миссией-помещением (s6/s7) исключены: там цель по замыслу не детектируется, и "
               "это ведро давало бы гарантированный ложноположительный вердикт.",
    },
    "planner": {
        "order": 5,
        "test": _rule_planner,
        "rule": "progress stalled and (TURN share > churn_turn_share or one action >= "
                "churn_dominant_share of >= churn_min_steps steps)",
        "why": "Цель видели, но политика крутится на месте или зациклилась на одном действии - "
               "это отказ принятия решений.",
    },
    "execution": {
        "order": 6,
        "test": _rule_execution,
        "rule": "(progress stalled and path_length_m >= execution_min_path_m) or "
                "plan_failures >= execution_min_plan_failures or "
                "degraded_events >= execution_min_degraded",
        "why": "План правдоподобен, но тело его не выполняет: километраж растёт без прогресса, "
               "либо доминируют отказы nav/skill.",
    },
    "timeout": {
        "order": 7,
        "test": _rule_timeout,
        "rule": "timed out (outcome == 'timeout' or duration >= timeout_s), progress still rising "
                "in the last (1 - progress_tail_fraction) of the episode",
        "why": "Патологии нет, не хватило времени. Это про бюджет сценария, а не про качество "
               "планировщика.",
    },
    "unattributed": {
        "order": 8,
        "test": _rule_unattributed,
        "rule": "fallback",
        "why": "Остаток. Должен быть пустым; ненулевое значение означает, что таблицу правил "
               "нужно доработать.",
    },
}


# --------------------------------------------------------------------------------------------
# Tiny helpers. Everything that touches a record goes through these so a malformed value degrades
# into a default instead of an exception.
# --------------------------------------------------------------------------------------------

def _get(rec, key, default, missing=None):
    """Read rec[key], substituting `default` when absent/None and counting the miss in `missing`."""
    if not isinstance(rec, dict) or key not in rec or rec[key] is None:
        if missing is not None:
            missing[key] = missing.get(key, 0) + 1
        return default
    return rec[key]


def _f(value, default=0.0):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    if math.isnan(out) or math.isinf(out):
        return default
    return out


def _i(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        try:
            return int(float(value))
        except (TypeError, ValueError):
            return default


def _b(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "y", "on")
    return bool(value)


def _as_count(value):
    """Counters in the record may be an int or the list of events itself - accept both."""
    if isinstance(value, bool):
        return 1 if value else 0
    if isinstance(value, (int, float)):
        return _i(value)
    if isinstance(value, (list, tuple, dict, set)):
        return len(value)
    return 0


def _as_dict(value):
    return value if isinstance(value, dict) else {}


def _as_list(value):
    if isinstance(value, (list, tuple)):
        return list(value)
    if isinstance(value, (str, bytes)):
        return [value]
    return []


def _mean(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    return sum(vals) / float(len(vals))


def _rate(hits, total):
    if not total:
        return None
    return float(hits) / float(total)


def _round(value, nd=4):
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, (int,)):
        return value
    try:
        return round(float(value), nd)
    except (TypeError, ValueError):
        return value


def _s(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return "%.4g" % value
    return str(value)


def _fmt(value, nd=3):
    if value is None:
        return "-"
    if isinstance(value, bool):
        return "да" if value else "нет"
    if isinstance(value, int):
        return str(value)
    try:
        return ("%." + str(nd) + "f") % float(value)
    except (TypeError, ValueError):
        return str(value)


def _pct(value):
    if value is None:
        return "-"
    return "%.0f%%" % (100.0 * value)


# --------------------------------------------------------------------------------------------
# Loading and normalisation
# --------------------------------------------------------------------------------------------

def _target_label_from_mission(mission):
    """Best-effort extraction of the detector label the mission is about."""
    text = (mission or "").strip().lower()
    if not text:
        return ""
    for word in TARGET_VOCAB:
        if word in text:
            return word
    # A bare label such as "cup" published straight onto /vlm_mission.
    if len(text.split()) == 1:
        return text
    return text


def _is_saw_subgoal(subgoal_id):
    return bool(SAW_SUBGOAL_RE.search(str(subgoal_id or "").lower()))


def _stalled(ep):
    """Progress stood still: not complete, and nothing completed in the tail of the episode."""
    if ep["ordered_progress"] >= THRESHOLDS["progress_complete"]:
        return False
    return not ep["progress_rising_at_end"]


def _camera_perturbed(perturbation_id, explicit, lights_off, overlay_index):
    """Did this episode run under a perturbation that degrades what the camera sees?"""
    if explicit is not None:
        return _b(explicit)
    pid = str(perturbation_id or "").lower()
    if pid in overlay_index:
        return overlay_index[pid]
    if lights_off:
        # Killing world lights degrades the image just as surely as a smudge filter does.
        return True
    if not pid or pid == NO_PERTURBATION:
        return False
    return any(tok in pid for tok in CAMERA_PERTURBATION_TOKENS)


def normalise(rec, source_file, missing, overlay_index):
    """Turn a raw record into the flat dict every later stage consumes."""
    ep = {}
    for key in sorted(FIELD_DEFAULTS):
        ep[key] = _get(rec, key, FIELD_DEFAULTS[key], missing)
    for key in sorted(OPTIONAL_FIELDS):
        # Optional fields are read without counting: their absence is expected, not a defect.
        ep[key] = _get(rec, key, OPTIONAL_FIELDS[key], None)

    ep["source_file"] = source_file
    ep["schema_version"] = _i(ep["schema_version"])
    ep["scenario_id"] = str(ep["scenario_id"])
    ep["perturbation_id"] = str(ep["perturbation_id"]) or NO_PERTURBATION
    ep["planner_label"] = str(ep["planner_label"])
    ep["seed"] = _i(ep["seed"], -1)
    ep["mission"] = str(ep["mission"])
    ep["outcome"] = str(ep["outcome"]).strip().lower()
    ep["outcome_reason"] = str(ep["outcome_reason"])
    ep["pose_source"] = str(ep["pose_source"])
    ep["first_failure_subgoal"] = str(ep["first_failure_subgoal"] or "")

    ep["n_subgoals"] = _i(ep["n_subgoals"])
    ep["ordered_progress"] = _f(ep["ordered_progress"])
    ep["unordered_progress"] = _f(ep["unordered_progress"])
    ep["progress_auc"] = _f(ep["progress_auc"])
    ep["wall_duration_s"] = _f(ep["wall_duration_s"])
    ep["sim_duration_s"] = _f(ep["sim_duration_s"])
    ep["timeout_s"] = _f(ep["timeout_s"])
    ep["path_length_m"] = _f(ep["path_length_m"])
    ep["max_path_m"] = _f(ep["max_path_m"])
    ep["collided"] = _b(ep["collided"])
    ep["vlm_steps"] = _i(ep["vlm_steps"])
    ep["detect_all_calls"] = _as_count(ep["detect_all_calls"])
    ep["plan_failures"] = _as_count(ep["plan_failures"])
    ep["degraded_events"] = _as_count(ep["degraded_events"])
    ep["notes_events"] = _as_count(ep["notes_events"])
    ep["subgoal_times"] = _as_dict(ep["subgoal_times"])
    ep["action_histogram"] = _as_dict(ep["action_histogram"])
    # Accept both shapes: the runner emits a mapping, but a hand-written or older record may
    # carry a plain list of names. Count is what the report uses either way.
    spawned = ep["spawned"]
    ep["spawned"] = _as_dict(spawned) if isinstance(spawned, dict) else \
        dict((str(n), {}) for n in _as_list(spawned))
    ep["diagnoses"] = sorted(str(x) for x in _as_list(ep["diagnoses"]))
    ep["lights_off"] = sorted(str(x) for x in _as_list(ep["lights_off"]))

    if not ep["episode_key"]:
        ep["episode_key"] = "%s__%s__%s__seed%d" % (
            ep["scenario_id"], ep["perturbation_id"], ep["planner_label"], ep["seed"])
    ep["episode_key"] = str(ep["episode_key"])

    # ---- derived outcome flags -------------------------------------------------------------
    ep["success"] = 1 if ep["outcome"] == OUTCOME_SUCCESS else 0
    duration = ep["wall_duration_s"] or ep["sim_duration_s"]
    hit_clock = bool(ep["timeout_s"] > 0.0 and duration >= ep["timeout_s"])
    ep["timed_out"] = bool(
        ep["outcome"] == OUTCOME_TIMEOUT
        or "timeout" in ep["outcome_reason"].lower()
        or (hit_clock and not ep["success"]))
    ep["duration_s"] = duration
    ep["path_over_budget"] = bool(ep["max_path_m"] > 0.0 and ep["path_length_m"] > ep["max_path_m"])
    ep["n_subgoals_done"] = len(ep["subgoal_times"])

    # ---- derived action-histogram statistics ------------------------------------------------
    hist = {}
    for name, count in ep["action_histogram"].items():
        hist[str(name)] = _as_count(count)
    total = sum(hist.values())
    ep["hist_total"] = total
    if total > 0:
        turn = sum(c for n, c in hist.items() if TURN_ACTION_RE.search(n))
        ep["turn_share"] = float(turn) / float(total)
        top = sorted(hist.items(), key=lambda kv: (-kv[1], kv[0]))[0]
        ep["dominant_action"] = top[0]
        ep["dominant_share"] = float(top[1]) / float(total)
    else:
        ep["turn_share"] = 0.0
        ep["dominant_action"] = ""
        ep["dominant_share"] = 0.0

    # ---- was progress still rising when the episode ended? ----------------------------------
    times = []
    for value in ep["subgoal_times"].values():
        times.append(_f(value, -1.0))
    times = [t for t in times if t >= 0.0]
    last_t = max(times) if times else None
    ep["last_subgoal_t"] = last_t
    if ep["ordered_progress"] >= THRESHOLDS["progress_complete"]:
        ep["progress_rising_at_end"] = True
    elif last_t is None or duration <= 0.0:
        ep["progress_rising_at_end"] = False
    else:
        ep["progress_rising_at_end"] = last_t > THRESHOLDS["progress_tail_fraction"] * duration

    # ---- grounding evidence ------------------------------------------------------------------
    ep["target_label"] = str(ep["target_label"] or _target_label_from_mission(ep["mission"]))
    seen = ep["seen_labels"]
    if isinstance(seen, (list, tuple, set)):
        seen_norm = set(str(x).strip().lower() for x in seen)
        ep["seen_labels"] = sorted(seen_norm)
        ep["saw_target"] = bool(ep["target_label"].strip().lower() in seen_norm)
    else:
        ep["seen_labels"] = None
        if ep["first_failure_subgoal"] and _is_saw_subgoal(ep["first_failure_subgoal"]):
            # The episode died at the detection step: by construction the label never came back.
            ep["saw_target"] = False
        elif ep["detect_all_calls"] == 0 and ep["ordered_progress"] <= 0.0:
            ep["saw_target"] = False
        else:
            # Optimistic default: without evidence we do NOT blame grounding.
            ep["saw_target"] = True

    ep["camera_perturbed"] = _camera_perturbed(
        ep["perturbation_id"], ep["camera_perturbed"], ep["lights_off"], overlay_index)
    ep["lights_off_n"] = len(ep["lights_off"])
    ep["spawned_n"] = len(ep["spawned"])
    ep["diagnoses_tags"] = " ".join(ep["diagnoses"])
    ep["diagnoses"] = sorted(str(x) for x in _as_list(ep["diagnoses"]))
    ep["scenario_title"] = str(ep["scenario_title"] or "")
    ep["attribution"] = ""
    ep["attribution_reason"] = ""
    return ep


def load_overlay_index(perturbation_dir):
    """Optionally read perturbation overlay YAMLs to learn which ones touch the camera.

    Returns (index, note). The index maps perturbation id -> bool. PyYAML is imported lazily so the
    tool keeps working on a machine without it.
    """
    if not perturbation_dir:
        return {}, ""
    path = os.path.expanduser(perturbation_dir)
    if not os.path.isdir(path):
        return {}, "перегрузки возмущений: каталог %s не найден, использована эвристика по имени" % path
    try:
        import yaml
    except ImportError:
        return {}, "перегрузки возмущений: PyYAML не установлен, использована эвристика по имени"
    index = {}
    for name in sorted(os.listdir(path)):
        if not name.endswith((".yaml", ".yml")):
            continue
        full = os.path.join(path, name)
        try:
            with open(full, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle)
        except Exception:
            continue
        if not isinstance(data, dict):
            continue
        pid = str(data.get("id") or os.path.splitext(name)[0]).lower()
        camera = data.get("camera")
        touches = False
        if isinstance(camera, dict):
            for key, value in camera.items():
                if key == "darkness":
                    touches = touches or _f(value, 1.0) < 1.0
                else:
                    touches = touches or _f(value, 0.0) > 0.0
        if _as_list(data.get("lights_off")):
            touches = True
        index[pid] = touches
    return index, "перегрузки возмущений прочитаны из %s (%d шт.)" % (path, len(index))


def collect_files(in_dir, skip_dirs):
    """All *.json under in_dir (recursive), excluding anything under skip_dirs. *.jsonl ignored."""
    root = os.path.expanduser(in_dir)
    found = []
    skip_real = [os.path.realpath(d) for d in skip_dirs]
    for path in glob.iglob(os.path.join(root, "**", "*.json"), recursive=True):
        real = os.path.realpath(path)
        if any(real.startswith(s + os.sep) or real == s for s in skip_real):
            continue
        found.append(path)
    return sorted(found, key=lambda p: p.replace("\\", "/"))


def load_episodes(in_dir, skip_dirs, overlay_index):
    """Load every episode record. Returns (episodes, skipped, missing_counts)."""
    episodes = []
    skipped = []
    missing = {}
    for path in collect_files(in_dir, skip_dirs):
        rel = os.path.relpath(path, os.path.expanduser(in_dir)).replace("\\", "/")
        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except ValueError as exc:
            skipped.append({"file": rel, "reason": "не разбирается как JSON: %s" % exc})
            continue
        except OSError as exc:
            skipped.append({"file": rel, "reason": "не читается: %s" % exc})
            continue
        if not isinstance(data, dict):
            skipped.append({"file": rel, "reason": "верхний уровень не объект (%s)" % type(data).__name__})
            continue
        if "schema_version" not in data:
            skipped.append({"file": rel, "reason": "нет поля schema_version - не запись эпизода"})
            continue
        episodes.append(normalise(data, rel, missing, overlay_index))
    episodes.sort(key=lambda e: (e["scenario_id"], e["perturbation_id"], e["planner_label"],
                                 e["seed"], e["episode_key"]))
    return episodes, skipped, missing


# --------------------------------------------------------------------------------------------
# Attribution context and pass
# --------------------------------------------------------------------------------------------

class AttributionContext(object):
    """Cross-episode references the rule table needs (currently: the p_none baseline progress)."""

    def __init__(self, episodes):
        self.by_triple = {}
        self.by_scenario_planner = {}
        for ep in episodes:
            if ep["perturbation_id"] != NO_PERTURBATION:
                continue
            self.by_triple[(ep["scenario_id"], ep["seed"], ep["planner_label"])] = \
                ep["ordered_progress"]
            key = (ep["scenario_id"], ep["planner_label"])
            self.by_scenario_planner.setdefault(key, []).append(ep["ordered_progress"])

    def reference_progress(self, ep):
        """ordered_progress of the undisturbed run: same scenario+seed+planner, else the mean."""
        exact = self.by_triple.get((ep["scenario_id"], ep["seed"], ep["planner_label"]))
        if exact is not None:
            return exact
        pool = self.by_scenario_planner.get((ep["scenario_id"], ep["planner_label"]))
        if pool:
            return _mean(pool)
        return None


def attribute_failures(episodes):
    """Assign exactly one bucket to every non-successful episode. Returns the bucket counter."""
    ctx = AttributionContext(episodes)
    counts = dict((bucket, 0) for bucket in ATTRIBUTION_ORDER)
    for ep in episodes:
        if ep["success"]:
            ep["attribution"] = ""
            ep["attribution_reason"] = ""
            continue
        for bucket in ATTRIBUTION_ORDER:
            reason = ATTRIBUTION_RULES[bucket]["test"](ep, ctx)
            if reason:
                ep["attribution"] = bucket
                ep["attribution_reason"] = reason
                counts[bucket] += 1
                break
    return counts


# --------------------------------------------------------------------------------------------
# B. Aggregates
# --------------------------------------------------------------------------------------------

AGGREGATE_METRICS = (
    "success_rate", "timeout_rate", "collision_rate", "mean_ordered_progress",
    "mean_unordered_progress", "mean_progress_auc", "mean_path_length_m", "mean_vlm_steps",
    "mean_detect_all_calls", "mean_plan_failures",
)


def compute_aggregates(episodes):
    groups = {}
    for ep in episodes:
        key = (ep["scenario_id"], ep["perturbation_id"], ep["planner_label"])
        groups.setdefault(key, []).append(ep)
    rows = []
    for key in sorted(groups):
        eps = groups[key]
        n = len(eps)
        row = {
            "scenario_id": key[0],
            "perturbation_id": key[1],
            "planner_label": key[2],
            "n": n,
            "success_rate": _rate(sum(e["success"] for e in eps), n),
            "timeout_rate": _rate(sum(1 for e in eps if e["timed_out"]), n),
            "collision_rate": _rate(sum(1 for e in eps if e["collided"]), n),
            "mean_ordered_progress": _mean([e["ordered_progress"] for e in eps]),
            "mean_unordered_progress": _mean([e["unordered_progress"] for e in eps]),
            "mean_progress_auc": _mean([e["progress_auc"] for e in eps]),
            "mean_path_length_m": _mean([e["path_length_m"] for e in eps]),
            "mean_vlm_steps": _mean([e["vlm_steps"] for e in eps]),
            "mean_detect_all_calls": _mean([e["detect_all_calls"] for e in eps]),
            "mean_plan_failures": _mean([e["plan_failures"] for e in eps]),
        }
        rows.append(row)
    return rows


# --------------------------------------------------------------------------------------------
# C. Paired planner deltas - the headline result
# --------------------------------------------------------------------------------------------

def epsilon_for(n_subgoals):
    """Dead-band = one subgoal. See the module docstring for why."""
    if n_subgoals and n_subgoals > 0:
        return 1.0 / float(n_subgoals)
    # Unknown resolution: widen the dead-band to the whole range so nothing is claimed. The pair is
    # flagged in the data-quality section.
    return 1.0


def verdict_for(delta, eps):
    """Classify a paired delta against the one-subgoal dead-band.

    The comparison is >= (with a float tolerance), not >. eps IS one subgoal, so gaining
    exactly one subgoal must count as benefit -- that is the smallest change the metric can
    even represent, and calling it "below the resolution of the metric" is wrong.

    This is not hypothetical. The first real paired run of s1 (3 subgoals, eps = 1/3) had
    the VLM arm reach one subgoal that flat_mock never reached: delta = +0.3333 against
    eps = 0.3333. With a strict >, that genuine gain was reported as neutral, and on any
    3-subgoal scenario the planner would have needed TWO extra subgoals before the report
    admitted it had helped at all.

    The comparison is done in SUBGOAL UNITS (delta / eps) rather than on the raw floats,
    because the runner rounds ordered_progress to 4 decimals when it writes the record: a
    one-subgoal gain on a 3-subgoal scenario is stored as 0.3333 while eps is a full
    0.333333..., so a direct >= would miss it by 3e-5 and silently report neutral forever.
    """
    if eps <= 0.0:
        return "neutral"
    units = delta / eps
    if units >= 1.0 - 1e-3:
        return "benefit"
    if units <= -1.0 + 1e-3:
        return "harm"
    return "neutral"


def compute_pairs(episodes, baseline, treatment):
    """Pair episodes on (scenario_id, perturbation_id, seed) across the two planner labels."""
    buckets = {}
    others = {}
    duplicates = []
    for ep in episodes:
        key = (ep["scenario_id"], ep["perturbation_id"], ep["seed"])
        if ep["planner_label"] == baseline:
            slot = "baseline"
        elif ep["planner_label"] == treatment:
            slot = "treatment"
        else:
            others[ep["planner_label"]] = others.get(ep["planner_label"], 0) + 1
            continue
        entry = buckets.setdefault(key, {"baseline": [], "treatment": []})
        entry[slot].append(ep)

    pairs = []
    unpaired = []
    bad_epsilon = []
    for key in sorted(buckets):
        entry = buckets[key]
        for slot in ("baseline", "treatment"):
            entry[slot].sort(key=lambda e: e["episode_key"])
            if len(entry[slot]) > 1:
                # Deterministic choice: first by episode_key. The rest are reported, not averaged,
                # because a repeated triple usually means the sweep was restarted.
                for extra in entry[slot][1:]:
                    duplicates.append({
                        "scenario_id": key[0], "perturbation_id": key[1], "seed": key[2],
                        "planner_label": extra["planner_label"],
                        "episode_key": extra["episode_key"], "source_file": extra["source_file"],
                    })
        base = entry["baseline"][0] if entry["baseline"] else None
        treat = entry["treatment"][0] if entry["treatment"] else None
        if base is None or treat is None:
            lone = base or treat
            unpaired.append({
                "scenario_id": key[0], "perturbation_id": key[1], "seed": key[2],
                "planner_label": lone["planner_label"], "episode_key": lone["episode_key"],
                "missing_side": treatment if treat is None else baseline,
                "source_file": lone["source_file"],
            })
            continue
        n_sub = treat["n_subgoals"] or base["n_subgoals"]
        if not n_sub:
            bad_epsilon.append("%s/%s/seed%d" % (key[0], key[1], key[2]))
        eps = epsilon_for(n_sub)
        delta_progress = treat["ordered_progress"] - base["ordered_progress"]
        pairs.append({
            "scenario_id": key[0],
            "perturbation_id": key[1],
            "seed": key[2],
            "n_subgoals": n_sub,
            "epsilon": eps,
            "baseline_key": base["episode_key"],
            "treatment_key": treat["episode_key"],
            "ordered_progress_baseline": base["ordered_progress"],
            "ordered_progress_treatment": treat["ordered_progress"],
            "delta_progress": delta_progress,
            "verdict": verdict_for(delta_progress, eps),
            "success_baseline": base["success"],
            "success_treatment": treat["success"],
            "delta_success": treat["success"] - base["success"],
            "delta_path_length_m": treat["path_length_m"] - base["path_length_m"],
            "delta_vlm_steps": treat["vlm_steps"] - base["vlm_steps"],
        })
    pairs.sort(key=lambda p: (p["scenario_id"], p["perturbation_id"], p["seed"]))
    unpaired.sort(key=lambda u: (u["scenario_id"], u["perturbation_id"], u["seed"],
                                 u["planner_label"]))
    duplicates.sort(key=lambda d: (d["scenario_id"], d["perturbation_id"], d["seed"],
                                   d["planner_label"], d["episode_key"]))
    return pairs, unpaired, duplicates, others, sorted(bad_epsilon)


def summarise_pairs(pairs, label):
    n = len(pairs)
    benefit = sum(1 for p in pairs if p["verdict"] == "benefit")
    harm = sum(1 for p in pairs if p["verdict"] == "harm")
    neutral = sum(1 for p in pairs if p["verdict"] == "neutral")
    return {
        "group": label,
        "n_pairs": n,
        "mean_delta_progress": _mean([p["delta_progress"] for p in pairs]),
        "mean_delta_success": _mean([float(p["delta_success"]) for p in pairs]),
        "mean_delta_path_length_m": _mean([p["delta_path_length_m"] for p in pairs]),
        "benefit": benefit,
        "neutral": neutral,
        "harm": harm,
        "benefit_rate": _rate(benefit, n),
        "neutral_rate": _rate(neutral, n),
        "harm_rate": _rate(harm, n),
        "epsilon_min": min([p["epsilon"] for p in pairs]) if pairs else None,
        "epsilon_max": max([p["epsilon"] for p in pairs]) if pairs else None,
    }


def summarise_pair_groups(pairs):
    """Overall summary plus one summary per (scenario_id, perturbation_id)."""
    groups = {}
    for pair in pairs:
        groups.setdefault((pair["scenario_id"], pair["perturbation_id"]), []).append(pair)
    rows = []
    for key in sorted(groups):
        row = summarise_pairs(groups[key], "%s / %s" % key)
        row["scenario_id"] = key[0]
        row["perturbation_id"] = key[1]
        rows.append(row)
    overall = summarise_pairs(pairs, "ВСЕ ПАРЫ")
    overall["scenario_id"] = "*"
    overall["perturbation_id"] = "*"
    return overall, rows


# --------------------------------------------------------------------------------------------
# D. Perturbation sensitivity
# --------------------------------------------------------------------------------------------

def compute_sensitivity(episodes):
    """Mean ordered_progress per (planner, perturbation), and the cost relative to p_none.

    The delta is computed on the INTERSECTION of scenarios that the planner ran both under the
    perturbation and under p_none; otherwise a perturbation that was only ever run on the hardest
    scenario would look catastrophic for the wrong reason.
    """
    index = {}
    for ep in episodes:
        index.setdefault(ep["planner_label"], {}) \
             .setdefault(ep["perturbation_id"], {}) \
             .setdefault(ep["scenario_id"], []).append(ep)
    rows = []
    for planner in sorted(index):
        by_pert = index[planner]
        base_by_scen = by_pert.get(NO_PERTURBATION, {})
        for pert in sorted(by_pert):
            by_scen = by_pert[pert]
            all_eps = [e for scen in sorted(by_scen) for e in by_scen[scen]]
            common = sorted(set(by_scen) & set(base_by_scen))
            here = [e["ordered_progress"] for scen in common for e in by_scen[scen]]
            ref = [e["ordered_progress"] for scen in common for e in base_by_scen[scen]]
            mean_here = _mean(here)
            mean_ref = _mean(ref)
            delta = None
            if mean_here is not None and mean_ref is not None:
                delta = mean_here - mean_ref
            rows.append({
                "planner_label": planner,
                "perturbation_id": pert,
                "n": len(all_eps),
                "mean_ordered_progress": _mean([e["ordered_progress"] for e in all_eps]),
                "success_rate": _rate(sum(e["success"] for e in all_eps), len(all_eps)),
                "n_common_scenarios": len(common),
                "mean_ordered_progress_common": mean_here,
                "ref_p_none_common": mean_ref,
                "delta_vs_p_none": delta,
            })
    return rows


# --------------------------------------------------------------------------------------------
# E. Attribution breakdown tables
# --------------------------------------------------------------------------------------------

def compute_attribution_tables(episodes):
    by_bucket = dict((b, 0) for b in ATTRIBUTION_ORDER)
    by_planner = {}
    by_scenario = {}
    for ep in episodes:
        if not ep["attribution"]:
            continue
        by_bucket[ep["attribution"]] += 1
        by_planner.setdefault(ep["planner_label"], dict((b, 0) for b in ATTRIBUTION_ORDER))
        by_planner[ep["planner_label"]][ep["attribution"]] += 1
        skey = (ep["scenario_id"], ep["perturbation_id"])
        by_scenario.setdefault(skey, dict((b, 0) for b in ATTRIBUTION_ORDER))
        by_scenario[skey][ep["attribution"]] += 1
    return by_bucket, by_planner, by_scenario


# --------------------------------------------------------------------------------------------
# Markdown helpers - the report must stay readable in a ~110 column terminal.
# --------------------------------------------------------------------------------------------

MD_MAX_WIDTH = 110
MD_MIN_COL = 5


def _clip(text, width):
    if width <= 0:
        return ""
    if len(text) <= width:
        return text + " " * (width - len(text))
    if width == 1:
        return "…"
    return text[:width - 1] + "…"


def md_table(headers, rows):
    """Render a Markdown table, shrinking the widest column until the line fits MD_MAX_WIDTH."""
    cols = len(headers)
    if cols == 0:
        return []
    body = [[_s(c) for c in row] + [""] * (cols - len(row)) for row in rows]
    cells = [[_s(h) for h in headers]] + body
    widths = []
    for i in range(cols):
        widths.append(max(1, max(len(row[i]) for row in cells)))

    def line_width():
        return sum(widths) + 3 * cols + 1

    guard = 0
    while line_width() > MD_MAX_WIDTH and guard < 4000:
        guard += 1
        widest = max(range(cols), key=lambda i: (widths[i], -i))
        if widths[widest] <= MD_MIN_COL:
            break
        widths[widest] -= 1

    out = ["| " + " | ".join(_clip(_s(headers[i]), widths[i]) for i in range(cols)) + " |"]
    out.append("|" + "|".join("-" * (widths[i] + 2) for i in range(cols)) + "|")
    for row in body:
        out.append("| " + " | ".join(_clip(row[i], widths[i]) for i in range(cols)) + " |")
    return out


def wrap(text, width=100):
    """Minimal greedy wrapper (textwrap would do, but this keeps punctuation predictable)."""
    words = text.split()
    lines = []
    current = ""
    for word in words:
        if not current:
            current = word
        elif len(current) + 1 + len(word) <= width:
            current += " " + word
        else:
            lines.append(current)
            current = word
    if current:
        lines.append(current)
    return lines


# --------------------------------------------------------------------------------------------
# Report assembly
# --------------------------------------------------------------------------------------------

EPISODE_COLUMNS = (
    "episode_key", "scenario_id", "perturbation_id", "planner_label", "seed", "outcome",
    "outcome_reason", "success", "timed_out", "collided", "n_subgoals", "n_subgoals_done",
    "ordered_progress", "unordered_progress", "progress_auc", "first_failure_subgoal",
    "duration_s", "wall_duration_s", "sim_duration_s", "timeout_s", "path_length_m", "max_path_m",
    "path_over_budget", "vlm_steps", "detect_all_calls", "plan_failures", "degraded_events",
    "notes_events", "hist_total", "turn_share", "dominant_action", "dominant_share",
    "last_subgoal_t", "progress_rising_at_end", "target_label", "saw_target", "camera_perturbed",
    "lights_off_n", "spawned_n", "pose_source", "schema_version", "mission", "diagnoses_tags",
    "attribution", "attribution_reason", "source_file",
)


def episode_row(ep):
    row = {}
    for col in EPISODE_COLUMNS:
        value = ep.get(col)
        if isinstance(value, float):
            value = _round(value, 4)
        row[col] = value
    return row


def write_csv(path, columns, rows):
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(list(columns))
        for row in rows:
            writer.writerow([_s(row.get(c)) for c in columns])


def write_json(path, payload):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, ensure_ascii=False)
        handle.write("\n")


def build_markdown(state):
    """Assemble the Russian-prose report. Metric names stay in English on purpose."""
    md = []
    add = md.append

    baseline = state["baseline"]
    treatment = state["treatment"]
    episodes = state["episodes"]
    overall = state["paired_overall"]

    # ---------------------------------------------------------------- header
    add("# Диагностический отчёт: house benchmark")
    add("")
    add("Сгенерировано `house_benchmark_report.py` %s." % state["generated_at"])
    add("")
    add("- вход: `%s`" % state["in_dir"])
    add("- эпизодов загружено: **%d**, файлов пропущено: **%d**" %
        (len(episodes), len(state["skipped"])))
    add("- baseline planner: **%s**, treatment planner: **%s**" % (baseline, treatment))
    add("- пар собрано: **%d**, непарных эпизодов: **%d**" %
        (len(state["pairs"]), len(state["unpaired"])))
    add("")

    # ---------------------------------------------------------------- house
    add("## 1. Стенд")
    add("")
    for line in HOUSE_DESCRIPTION:
        add(line)
    add("")

    add("### Семь диагностик")
    add("")
    add("Набор сценариев разложен по семи осям. Каждая ось изолирует один способ сломаться,")
    add("поэтому провал в одном сценарии не размазывается по остальным.")
    add("")
    rows = []
    for idx, (tag, text) in enumerate(DIAGNOSTIC_AXES, start=1):
        rows.append([idx, tag, text])
    for line in md_table(["#", "ось", "что проверяет"], rows):
        add(line)
    add("")

    if state["scenarios_seen"]:
        add("Сценарии, фактически найденные в данных:")
        add("")
        srows = []
        for scen in state["scenarios_seen"]:
            info = state["scenario_info"][scen]
            srows.append([scen, info["title"], ", ".join(info["diagnoses"]) or "-",
                          info["n_subgoals"], info["n_episodes"]])
        for line in md_table(["scenario_id", "название", "diagnoses", "n_subgoals", "n_ep"], srows):
            add(line)
        add("")

    # ---------------------------------------------------------------- how to read
    add("## 2. Как читать метрики")
    add("")
    add("- `ordered_progress` - доля ПРЕФИКСА упорядоченного списка подцелей, которую робот прошёл.")
    add("  Величина дискретная, её квант равен `1 / n_subgoals`.")
    add("- `unordered_progress` - доля выполненных подцелей без учёта порядка (верхняя оценка).")
    add("- `progress_auc` - площадь под кривой прогресса по времени: награждает раннее продвижение.")
    add("- `success_rate` / `timeout_rate` / `collision_rate` - доли эпизодов в группе.")
    add("- `delta_*` - ПАРНАЯ разница treatment минус baseline при одинаковых")
    add("  (scenario_id, perturbation_id, seed).")
    add("")
    add("**Мёртвая зона (dead-band).** Пара классифицируется с порогом `epsilon = 1 / n_subgoals`:")
    add("`delta_progress >= +epsilon` - benefit, `<= -epsilon` - harm, иначе **neutral**. "
        "epsilon равен ОДНОЙ подцели, поэтому выигрыш ровно в одну подцель засчитывается: "
        "это минимальное изменение, которое метрика вообще способна выразить.")
    add("Разница меньше одной подцели физически не может означать «дошёл дальше»: это ниже")
    add("разрешения метрики, и такую пару мы честно считаем нейтральной, а не победой.")
    add("")

    # ---------------------------------------------------------------- data quality
    add("## 3. Качество данных")
    add("")
    add("- файлов `*.json` просмотрено: %d (`*.jsonl` игнорируются)" % state["n_files_seen"])
    add("- записей принято: %d" % len(episodes))
    add("- записей пропущено: %d" % len(state["skipped"]))
    if state["overlay_note"]:
        add("- %s" % state["overlay_note"])
    add("")
    if state["skipped"]:
        for line in md_table(["файл", "причина"],
                             [[s["file"], s["reason"]] for s in state["skipped"]]):
            add(line)
        add("")
    if state["missing"]:
        add("Отсутствующие поля (значение подставлено по умолчанию):")
        add("")
        mrows = [[k, state["missing"][k], _s(FIELD_DEFAULTS.get(k))]
                 for k in sorted(state["missing"])]
        for line in md_table(["поле", "записей без поля", "подставлено"], mrows):
            add(line)
        add("")
    else:
        add("Все обязательные поля присутствуют во всех записях.")
        add("")
    if state["duplicates"]:
        add("Дубликаты триплета (scenario, perturbation, seed, planner) - в пару взят первый по")
        add("`episode_key`, остальные перечислены ниже и в метрики не входят:")
        add("")
        for line in md_table(["scenario", "perturbation", "seed", "planner", "episode_key"],
                             [[d["scenario_id"], d["perturbation_id"], d["seed"],
                               d["planner_label"], d["episode_key"]] for d in state["duplicates"]]):
            add(line)
        add("")
    if state["other_planners"]:
        add("Эпизоды с другими planner_label (в парный анализ не входят): %s." %
            ", ".join("%s x%d" % (k, state["other_planners"][k])
                      for k in sorted(state["other_planners"])))
        add("")
    if state["bad_epsilon"]:
        add("ВНИМАНИЕ: у этих пар `n_subgoals == 0`, epsilon расширен до 1.0 (все дельты")
        add("нейтральны): %s." % ", ".join(state["bad_epsilon"]))
        add("")

    # ---------------------------------------------------------------- aggregates
    add("## 4. Агрегаты по (scenario, perturbation, planner)")
    add("")
    add("### 4.1 Исходы и прогресс")
    add("")
    arows = []
    for row in state["aggregates"]:
        arows.append([row["scenario_id"], row["perturbation_id"], row["planner_label"], row["n"],
                      _pct(row["success_rate"]), _pct(row["timeout_rate"]),
                      _pct(row["collision_rate"]), _fmt(row["mean_ordered_progress"])])
    for line in md_table(["scenario", "perturb", "planner", "n", "succ", "t/out", "coll", "ord_pg"],
                         arows):
        add(line)
    add("")
    add("### 4.2 Стоимость эпизода")
    add("")
    crows = []
    for row in state["aggregates"]:
        crows.append([row["scenario_id"], row["perturbation_id"], row["planner_label"],
                      _fmt(row["mean_unordered_progress"]), _fmt(row["mean_progress_auc"]),
                      _fmt(row["mean_path_length_m"], 1), _fmt(row["mean_vlm_steps"], 1),
                      _fmt(row["mean_detect_all_calls"], 1), _fmt(row["mean_plan_failures"], 1)])
    for line in md_table(["scenario", "perturb", "planner", "unord", "auc", "path", "steps",
                          "det", "planf"], crows):
        add(line)
    add("")

    # ---------------------------------------------------------------- paired
    add("## 5. Парные дельты планировщика (главный результат)")
    add("")
    add("Пары строятся по (scenario_id, perturbation_id, seed): один и тот же сценарий, то же")
    add("возмущение, тот же seed - меняется только planner_label (`%s` -> `%s`)." %
        (baseline, treatment))
    add("Порог нейтральности `epsilon = 1 / n_subgoals` - одна подцель.")
    add("")
    prows = []
    for row in state["paired_groups"]:
        prows.append([row["scenario_id"], row["perturbation_id"], row["n_pairs"],
                      _fmt(row["mean_delta_progress"]), _fmt(row["mean_delta_success"]),
                      _pct(row["benefit_rate"]), _pct(row["neutral_rate"]),
                      _pct(row["harm_rate"])])
    prows.append(["ИТОГО", "*", overall["n_pairs"], _fmt(overall["mean_delta_progress"]),
                  _fmt(overall["mean_delta_success"]), _pct(overall["benefit_rate"]),
                  _pct(overall["neutral_rate"]), _pct(overall["harm_rate"])])
    for line in md_table(["scenario", "perturb", "pairs", "d_prog", "d_succ", "benefit",
                          "neutral", "harm"], prows):
        add(line)
    add("")
    if overall["n_pairs"]:
        add("Итог: **mean delta_progress = %s**, **mean delta_success = %s**, "
            "benefit/neutral/harm = **%s / %s / %s**." % (
                _fmt(overall["mean_delta_progress"]), _fmt(overall["mean_delta_success"]),
                _pct(overall["benefit_rate"]), _pct(overall["neutral_rate"]),
                _pct(overall["harm_rate"])))
        add("Диапазон epsilon по парам: %s .. %s." %
            (_fmt(overall["epsilon_min"]), _fmt(overall["epsilon_max"])))
    else:
        add("Пар нет: в данных не встретились оба planner_label на одном триплете.")
    add("")
    add("### 5.1 Непарные эпизоды")
    add("")
    if state["unpaired"]:
        add("Эти эпизоды НЕ участвуют в парных метриках (у них нет партнёра). Их наличие означает,")
        add("что прогон неполон; `--strict` превращает это в ошибку выхода.")
        add("")
        for line in md_table(["scenario", "perturb", "seed", "есть", "нет"],
                             [[u["scenario_id"], u["perturbation_id"], u["seed"],
                               u["planner_label"], u["missing_side"]] for u in state["unpaired"]]):
            add(line)
    else:
        add("Непарных эпизодов нет - прогон полный.")
    add("")

    # ---------------------------------------------------------------- sensitivity
    add("## 6. Чувствительность к возмущениям")
    add("")
    add("Сколько стоит каждое возмущение относительно `p_none`. Дельта считается только по тем")
    add("сценариям, которые данный планировщик прошёл И под возмущением, И под `p_none`.")
    add("")
    srows = []
    for row in state["sensitivity"]:
        srows.append([row["planner_label"], row["perturbation_id"], row["n"],
                      _fmt(row["mean_ordered_progress"]), _pct(row["success_rate"]),
                      row["n_common_scenarios"], _fmt(row["mean_ordered_progress_common"]),
                      _fmt(row["ref_p_none_common"]), _fmt(row["delta_vs_p_none"])])
    for line in md_table(["planner", "perturb", "n", "ord_pg", "succ", "scn", "ord_c", "ref",
                          "delta"], srows):
        add(line)
    add("")

    # ---------------------------------------------------------------- attribution
    add("## 7. Атрибуция отказов")
    add("")
    add("Каждый неуспешный эпизод попадает РОВНО в одну корзину; правила применяются по порядку,")
    add("побеждает первое совпавшее. Таблица правил - в приложении A.")
    add("")
    n_fail = sum(state["attribution_counts"].values())
    brows = []
    for bucket in ATTRIBUTION_ORDER:
        count = state["attribution_counts"][bucket]
        brows.append([ATTRIBUTION_RULES[bucket]["order"], bucket, count,
                      _pct(_rate(count, n_fail))])
    brows.append(["", "ИТОГО неуспешных", n_fail, "100%" if n_fail else "-"])
    for line in md_table(["#", "bucket", "n", "доля"], brows):
        add(line)
    add("")
    if state["attribution_by_planner"]:
        add("### 7.1 По планировщикам")
        add("")
        headers = ["planner"] + [b[:6] for b in ATTRIBUTION_ORDER]
        arows = []
        for planner in sorted(state["attribution_by_planner"]):
            counts = state["attribution_by_planner"][planner]
            arows.append([planner] + [counts[b] for b in ATTRIBUTION_ORDER])
        for line in md_table(headers, arows):
            add(line)
        add("")
    if state["attribution_by_scenario"]:
        add("### 7.2 По сценариям и возмущениям")
        add("")
        headers = ["scenario", "perturb"] + [b[:5] for b in ATTRIBUTION_ORDER]
        arows = []
        for key in sorted(state["attribution_by_scenario"]):
            counts = state["attribution_by_scenario"][key]
            arows.append([key[0], key[1]] + [counts[b] for b in ATTRIBUTION_ORDER])
        for line in md_table(headers, arows):
            add(line)
        add("")
    if state["attribution_counts"]["unattributed"]:
        add("ВНИМАНИЕ: %d эпизодов не попали ни в одно правило. Таблицу правил нужно доработать." %
            state["attribution_counts"]["unattributed"])
        add("")

    # ---------------------------------------------------------------- what it means
    add("## 8. Что это значит")
    add("")
    add("_Шаблон интерпретации: заполняется человеком, цифры подставлены автоматически._")
    add("")
    add("**8.1 Даёт ли планировщик `%s` выигрыш над `%s`?**" % (treatment, baseline))
    add("")
    if overall["n_pairs"]:
        add("Средняя парная дельта прогресса %s при epsilon %s..%s; benefit %s, neutral %s, harm %s." % (
            _fmt(overall["mean_delta_progress"]), _fmt(overall["epsilon_min"]),
            _fmt(overall["epsilon_max"]), _pct(overall["benefit_rate"]),
            _pct(overall["neutral_rate"]), _pct(overall["harm_rate"])))
        for line in wrap(
                "Читать так: доля neutral - это доля прогонов, где смена планировщика не изменила "
                "результат НИ НА ОДНУ подцель. Если neutral доминирует, различие между "
                "планировщиками лежит ниже разрешения бенчмарка, и увеличивать число seed "
                "бессмысленно - нужно менять сценарии, а не выборку."):
            add(line)
    else:
        add("_Парных данных нет - вывод сделать нельзя._")
    add("")
    add("**8.2 Где именно ломается система?**")
    add("")
    if n_fail:
        top = sorted(((state["attribution_counts"][b], b) for b in ATTRIBUTION_ORDER),
                     key=lambda kv: (-kv[0], kv[1]))
        top_named = ", ".join("`%s` %d" % (b, c) for c, b in top if c)
        add("Крупнейшие корзины отказов: %s." % top_named)
        for line in wrap(
                "Читать так: `grounding` - чинить детектор и высоту/освещение цели; `planner` - "
                "чинить политику и промпт; `execution` - чинить nav2, локальный планировщик и "
                "сцепление с поверхностью; `perception` - это наше собственное возмущение, оно "
                "показывает запас прочности восприятия; `timeout` - вопрос к бюджету сценария, а "
                "не к качеству робота; `catastrophic` и `setup` - дефекты стенда и безопасности."):
            add(line)
    else:
        add("_Неуспешных эпизодов нет._")
    add("")
    add("**8.3 Какое возмущение дороже всего?**")
    add("")
    worst = None
    for row in state["sensitivity"]:
        if row["perturbation_id"] == NO_PERTURBATION or row["delta_vs_p_none"] is None:
            continue
        if worst is None or row["delta_vs_p_none"] < worst["delta_vs_p_none"]:
            worst = row
    if worst:
        add("Максимальная просадка: `%s` у планировщика `%s`, delta_vs_p_none = %s." %
            (worst["perturbation_id"], worst["planner_label"], _fmt(worst["delta_vs_p_none"])))
        for line in wrap(
                "Читать так: если просадка у treatment заметно меньше, чем у baseline, "
                "планировщик действительно добавляет устойчивость, а не только среднее качество."):
            add(line)
    else:
        add("_Данных по возмущениям недостаточно._")
    add("")
    add("**8.4 Что делать дальше?** (заполнить вручную)")
    add("")
    add("- [ ] ...")
    add("- [ ] ...")
    add("")

    # ---------------------------------------------------------------- appendix
    add("## Приложение A. Таблица правил атрибуции")
    add("")
    add("Правила применяются сверху вниз, побеждает первое совпавшее.")
    add("")
    for bucket in ATTRIBUTION_ORDER:
        rule = ATTRIBUTION_RULES[bucket]
        add("**%d. `%s`**" % (rule["order"], bucket))
        add("")
        for line in wrap("условие: " + rule["rule"]):
            add(line)
        for line in wrap("почему: " + rule["why"]):
            add(line)
        add("")
    add("### Пороги")
    add("")
    for line in md_table(["параметр", "значение"],
                         [[k, THRESHOLDS[k]] for k in sorted(THRESHOLDS)]):
        add(line)
    add("")
    add("## Приложение B. Файлы отчёта")
    add("")
    for line in md_table(["файл", "содержимое"],
                         [[name, desc] for name, desc in state["artifacts"]]):
        add(line)
    add("")
    return "\n".join(md) + "\n"


# --------------------------------------------------------------------------------------------
# Pipeline
# --------------------------------------------------------------------------------------------

AGGREGATE_COLUMNS = ("scenario_id", "perturbation_id", "planner_label", "n") + AGGREGATE_METRICS

PAIR_COLUMNS = (
    "scenario_id", "perturbation_id", "seed", "n_subgoals", "epsilon", "baseline_key",
    "treatment_key", "ordered_progress_baseline", "ordered_progress_treatment", "delta_progress",
    "verdict", "success_baseline", "success_treatment", "delta_success", "delta_path_length_m",
    "delta_vlm_steps",
)

PAIR_SUMMARY_COLUMNS = (
    "scenario_id", "perturbation_id", "group", "n_pairs", "mean_delta_progress",
    "mean_delta_success", "mean_delta_path_length_m", "benefit", "neutral", "harm",
    "benefit_rate", "neutral_rate", "harm_rate", "epsilon_min", "epsilon_max",
)

SENSITIVITY_COLUMNS = (
    "planner_label", "perturbation_id", "n", "mean_ordered_progress", "success_rate",
    "n_common_scenarios", "mean_ordered_progress_common", "ref_p_none_common", "delta_vs_p_none",
)

ATTRIBUTION_COLUMNS = (
    "episode_key", "scenario_id", "perturbation_id", "planner_label", "seed", "outcome",
    "ordered_progress", "attribution", "attribution_reason",
)


def run_pipeline(args):
    """Load -> compute -> write. Returns (state, exit_code)."""
    in_dir = os.path.abspath(os.path.expanduser(args.in_dir))
    out_dir = os.path.abspath(os.path.expanduser(args.out)) if args.out \
        else os.path.join(in_dir, "report")
    formats = set(args.formats)

    overlay_index, overlay_note = load_overlay_index(args.perturbation_dir)
    n_files_seen = len(collect_files(in_dir, [out_dir]))
    episodes, skipped, missing = load_episodes(in_dir, [out_dir], overlay_index)

    # ---- filters ---------------------------------------------------------------------------
    if args.scenarios:
        wanted = set(args.scenarios)
        episodes = [e for e in episodes if e["scenario_id"] in wanted]
    if args.perturbations:
        wanted = set(args.perturbations)
        episodes = [e for e in episodes if e["perturbation_id"] in wanted]

    attribution_counts = attribute_failures(episodes)
    aggregates = compute_aggregates(episodes)
    pairs, unpaired, duplicates, other_planners, bad_epsilon = compute_pairs(
        episodes, args.baseline, args.treatment)
    paired_overall, paired_groups = summarise_pair_groups(pairs)
    sensitivity = compute_sensitivity(episodes)
    by_bucket, by_planner, by_scenario = compute_attribution_tables(episodes)

    scenarios_seen = sorted(set(e["scenario_id"] for e in episodes))
    scenario_info = {}
    for scen in scenarios_seen:
        eps = [e for e in episodes if e["scenario_id"] == scen]
        titles = sorted(set(e["scenario_title"] for e in eps if e["scenario_title"]))
        diags = sorted(set(d for e in eps for d in e["diagnoses"]))
        scenario_info[scen] = {
            "title": titles[0] if titles else "-",
            "diagnoses": diags,
            "n_subgoals": max([e["n_subgoals"] for e in eps]),
            "n_episodes": len(eps),
        }

    artifacts = []
    if "csv" in formats:
        artifacts.extend([
            ("episodes.csv", "A. по одной строке на эпизод"),
            ("aggregates.csv", "B. агрегаты по (scenario, perturbation, planner)"),
            ("paired_deltas.csv", "C. одна строка на пару baseline/treatment"),
            ("paired_summary.csv", "C. сводка по парам + строка ИТОГО"),
            ("unpaired.csv", "C. эпизоды без партнёра"),
            ("sensitivity.csv", "D. чувствительность к возмущениям"),
            ("attribution.csv", "E. корзина отказа для каждого неуспешного эпизода"),
        ])
    if "json" in formats:
        artifacts.extend([
            ("episodes.json", "A. те же эпизоды в JSON"),
            ("summary.json", "B-E. все агрегаты одним объектом"),
        ])
    if "md" in formats:
        artifacts.append(("report.md", "F. отчёт целиком"))

    state = {
        "generated_at": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC"),
        "in_dir": in_dir,
        "out_dir": out_dir,
        "baseline": args.baseline,
        "treatment": args.treatment,
        "episodes": episodes,
        "skipped": skipped,
        "missing": missing,
        "n_files_seen": n_files_seen,
        "overlay_note": overlay_note,
        "aggregates": aggregates,
        "pairs": pairs,
        "unpaired": unpaired,
        "duplicates": duplicates,
        "other_planners": other_planners,
        "bad_epsilon": bad_epsilon,
        "paired_overall": paired_overall,
        "paired_groups": paired_groups,
        "sensitivity": sensitivity,
        "attribution_counts": by_bucket,
        "attribution_by_planner": by_planner,
        "attribution_by_scenario": by_scenario,
        "scenarios_seen": scenarios_seen,
        "scenario_info": scenario_info,
        "artifacts": artifacts,
        "written": [],
    }
    # attribute_failures() already filled the counters; keep the two views consistent.
    state["attribution_counts"] = attribution_counts

    if not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    episode_rows = [episode_row(e) for e in episodes]
    attribution_rows = [dict((c, episode_row(e).get(c)) for c in ATTRIBUTION_COLUMNS)
                        for e in episodes if e["attribution"]]

    def emit(name, writer):
        path = os.path.join(out_dir, name)
        writer(path)
        state["written"].append(path)

    if "csv" in formats:
        emit("episodes.csv", lambda p: write_csv(p, EPISODE_COLUMNS, episode_rows))
        emit("aggregates.csv", lambda p: write_csv(p, AGGREGATE_COLUMNS, [
            dict((k, _round(v)) for k, v in row.items()) for row in aggregates]))
        emit("paired_deltas.csv", lambda p: write_csv(p, PAIR_COLUMNS, [
            dict((k, _round(v)) for k, v in row.items()) for row in pairs]))
        emit("paired_summary.csv", lambda p: write_csv(p, PAIR_SUMMARY_COLUMNS, [
            dict((k, _round(v)) for k, v in row.items())
            for row in (paired_groups + [paired_overall])]))
        emit("unpaired.csv", lambda p: write_csv(
            p, ("scenario_id", "perturbation_id", "seed", "planner_label", "missing_side",
                "episode_key", "source_file"), unpaired))
        emit("sensitivity.csv", lambda p: write_csv(p, SENSITIVITY_COLUMNS, [
            dict((k, _round(v)) for k, v in row.items()) for row in sensitivity]))
        emit("attribution.csv", lambda p: write_csv(p, ATTRIBUTION_COLUMNS, attribution_rows))

    summary = {
        "generated_at": state["generated_at"],
        "in_dir": in_dir,
        "baseline_planner": args.baseline,
        "treatment_planner": args.treatment,
        "n_episodes": len(episodes),
        "n_files_seen": n_files_seen,
        "skipped_files": skipped,
        "missing_fields": missing,
        "scenarios": scenario_info,
        "aggregates": [dict((k, _round(v)) for k, v in row.items()) for row in aggregates],
        "paired": {
            "protocol": "pair on (scenario_id, perturbation_id, seed); "
                        "epsilon = 1 / n_subgoals dead-band",
            "overall": dict((k, _round(v)) for k, v in paired_overall.items()),
            "by_group": [dict((k, _round(v)) for k, v in row.items()) for row in paired_groups],
            "pairs": [dict((k, _round(v)) for k, v in row.items()) for row in pairs],
            "unpaired": unpaired,
            "duplicates": duplicates,
            "other_planner_labels": other_planners,
            "pairs_with_unknown_n_subgoals": bad_epsilon,
        },
        "sensitivity": [dict((k, _round(v)) for k, v in row.items()) for row in sensitivity],
        "attribution": {
            "order": list(ATTRIBUTION_ORDER),
            "rules": dict((b, {"order": ATTRIBUTION_RULES[b]["order"],
                               "rule": ATTRIBUTION_RULES[b]["rule"],
                               "why": ATTRIBUTION_RULES[b]["why"]})
                          for b in ATTRIBUTION_ORDER),
            "thresholds": THRESHOLDS,
            "counts": by_bucket,
            "by_planner": by_planner,
            "by_scenario": dict(("%s/%s" % k, v) for k, v in by_scenario.items()),
        },
    }
    state["summary"] = summary

    if "json" in formats:
        emit("episodes.json", lambda p: write_json(p, episode_rows))
        emit("summary.json", lambda p: write_json(p, summary))
    if "md" in formats:
        text = build_markdown(state)
        state["markdown"] = text

        def _write_md(path):
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(text)
        emit("report.md", _write_md)

    exit_code = 0
    if args.strict and unpaired:
        exit_code = 2
    return state, exit_code


# --------------------------------------------------------------------------------------------
# Self-test
# --------------------------------------------------------------------------------------------

def _mk_record(scenario, perturbation, planner, seed, n_subgoals, ordered_progress, outcome,
               **kwargs):
    """Fabricate one synthetic episode record with every documented field present."""
    duration = kwargs.get("wall_duration_s", 60.0)
    rec = {
        "schema_version": 1,
        "episode_key": "%s__%s__%s__seed%d" % (scenario, perturbation, planner, seed),
        "scenario_id": scenario,
        "perturbation_id": perturbation,
        "planner_label": planner,
        "seed": seed,
        "mission": "найди cup",
        "wall_duration_s": duration,
        "sim_duration_s": duration,
        "outcome": outcome,
        "outcome_reason": kwargs.get("outcome_reason", ""),
        "n_subgoals": n_subgoals,
        "ordered_progress": ordered_progress,
        "unordered_progress": kwargs.get("unordered_progress", ordered_progress),
        "progress_auc": kwargs.get("progress_auc", ordered_progress * 0.5),
        "first_failure_subgoal": kwargs.get("first_failure_subgoal", ""),
        "subgoal_times": kwargs.get("subgoal_times", {}),
        "path_length_m": kwargs.get("path_length_m", 5.0),
        "max_path_m": kwargs.get("max_path_m", 40.0),
        "timeout_s": kwargs.get("timeout_s", 60.0),
        "collided": kwargs.get("collided", False),
        "vlm_steps": kwargs.get("vlm_steps", 20),
        "action_histogram": kwargs.get("action_histogram", {"FORWARD": 12, "TURN_LEFT": 3}),
        "detect_all_calls": kwargs.get("detect_all_calls", 12),
        "plan_failures": kwargs.get("plan_failures", 0),
        "degraded_events": kwargs.get("degraded_events", 0),
        "notes_events": kwargs.get("notes_events", 1),
        "pose_source": "odom",
        "spawned": ["target"],
        "lights_off": kwargs.get("lights_off", []),
    }
    return rec


def _selftest_records():
    """Synthetic sweep with a hand-checkable answer. See _self_test() for the expected numbers."""
    a = "s1_far_target"
    b = "s5_dark_kitchen"
    c = "s7_setup_probe"
    recs = []

    # --- scenario A, p_none, 4 subgoals -> epsilon = 0.25 -------------------------------------
    # seed 1: 0.25 -> 1.00, delta +0.75  > +0.25  => benefit (and delta_success +1)
    recs.append(_mk_record(a, "p_none", "flat_mock", 1, 4, 0.25, "failed",
                           first_failure_subgoal="saw_cup", detect_all_calls=0,
                           subgoal_times={"in_hallway": 4.0}))
    recs.append(_mk_record(a, "p_none", "vlm", 1, 4, 1.00, "success",
                           subgoal_times={"in_hallway": 4.0, "saw_cup": 9.0,
                                          "in_kitchen": 20.0, "near_cup": 31.0}))
    # seed 2: 0.50 -> 0.70, delta +0.20  <= 0.25  => NEUTRAL (this is the dead-band test)
    recs.append(_mk_record(a, "p_none", "flat_mock", 2, 4, 0.50, "timeout",
                           first_failure_subgoal="in_kitchen", path_length_m=22.0,
                           plan_failures=0, action_histogram={"FORWARD": 30, "TURN_LEFT": 5},
                           subgoal_times={"in_hallway": 3.0, "saw_cup": 5.0}))
    recs.append(_mk_record(a, "p_none", "vlm", 2, 4, 0.70, "timeout",
                           first_failure_subgoal="near_cup", path_length_m=18.0,
                           subgoal_times={"in_hallway": 3.0, "saw_cup": 9.0,
                                          "in_kitchen": 57.0}))
    # seed 3: 0.75 -> 0.25, delta -0.50  < -0.25  => harm
    recs.append(_mk_record(a, "p_none", "flat_mock", 3, 4, 0.75, "timeout",
                           first_failure_subgoal="near_cup", path_length_m=8.0,
                           action_histogram={"FORWARD": 10, "TURN_LEFT": 4},
                           subgoal_times={"in_hallway": 2.0, "saw_cup": 6.0,
                                          "in_kitchen": 58.0}))
    recs.append(_mk_record(a, "p_none", "vlm", 3, 4, 0.25, "failed",
                           first_failure_subgoal="near_cup", path_length_m=1.2,
                           action_histogram={"TURN_LEFT": 40, "FORWARD": 5},
                           subgoal_times={"in_hallway": 3.0}))
    # seed 4: treatment only -> exactly one unpaired episode
    recs.append(_mk_record(a, "p_none", "vlm", 4, 4, 0.50, "timeout",
                           first_failure_subgoal="in_kitchen",
                           subgoal_times={"in_hallway": 3.0, "saw_cup": 57.0}))

    # --- scenario B: p_none vs a camera perturbation ------------------------------------------
    for planner in ("flat_mock", "vlm"):
        recs.append(_mk_record(b, "p_none", planner, 1, 4, 1.00, "success",
                               subgoal_times={"in_hallway": 3.0, "saw_cup": 8.0,
                                              "in_kitchen": 19.0, "near_cup": 28.0}))
        recs.append(_mk_record(b, "p_cam_smudge", planner, 1, 4, 0.25, "failed",
                               first_failure_subgoal="saw_cup", detect_all_calls=14,
                               subgoal_times={"in_hallway": 4.0}))

    # --- scenario C: setup failure vs collision, 2 subgoals -> epsilon = 0.5 ------------------
    recs.append(_mk_record(c, "p_none", "vlm", 1, 2, 0.00, "setup_failed",
                           outcome_reason="gz create service timed out"))
    recs.append(_mk_record(c, "p_none", "flat_mock", 1, 2, 0.50, "failed", collided=True,
                           subgoal_times={"in_hallway": 5.0}))
    return recs


def _self_test():
    """Fabricate a sweep, run the whole pipeline, assert the paired metrics come out right."""
    tmp = tempfile.mkdtemp(prefix="house_benchmark_selftest_")
    log = []

    def say(text):
        log.append(text)
        print(text)

    try:
        raw = os.path.join(tmp, "raw")
        os.makedirs(raw)
        for rec in _selftest_records():
            with open(os.path.join(raw, rec["episode_key"] + ".json"), "w",
                      encoding="utf-8") as handle:
                json.dump(rec, handle, ensure_ascii=False, indent=2)
        # Three decoys: broken JSON, a JSON object without schema_version, and a .jsonl that must
        # be ignored entirely (not even counted as skipped).
        with open(os.path.join(raw, "broken.json"), "w", encoding="utf-8") as handle:
            handle.write("{ this is not json ")
        with open(os.path.join(raw, "not_an_episode.json"), "w", encoding="utf-8") as handle:
            json.dump({"hello": "world"}, handle)
        with open(os.path.join(raw, "trace.jsonl"), "w", encoding="utf-8") as handle:
            handle.write('{"event": "step_start"}\n{"event": "mission_end"}\n')

        out = os.path.join(tmp, "report")
        args = argparse.Namespace(
            in_dir=raw, out=out, formats=["md", "csv", "json"], baseline="flat_mock",
            treatment="vlm", scenarios=[], perturbations=[], strict=False,
            perturbation_dir=None)
        state, code = run_pipeline(args)
        say("[self-test] pipeline exit code: %d" % code)

        # ---- loading ------------------------------------------------------------------------
        assert code == 0, "non-strict run must exit 0"
        assert len(state["episodes"]) == 13, "expected 13 episodes, got %d" % len(state["episodes"])
        assert len(state["skipped"]) == 2, "expected 2 skipped files, got %d" % len(state["skipped"])
        reasons = " ".join(s["reason"] for s in state["skipped"])
        assert "JSON" in reasons and "schema_version" in reasons, reasons
        assert not any(s["file"].endswith(".jsonl") for s in state["skipped"]), "*.jsonl must be ignored"
        assert state["missing"] == {}, "no field should be missing: %r" % state["missing"]
        say("[self-test] loaded %d episodes, skipped %d files, ignored *.jsonl"
            % (len(state["episodes"]), len(state["skipped"])))

        # ---- C: paired deltas ----------------------------------------------------------------
        assert len(state["pairs"]) == 6, "expected 6 pairs, got %d" % len(state["pairs"])
        assert len(state["unpaired"]) == 1, "expected 1 unpaired, got %d" % len(state["unpaired"])
        lone = state["unpaired"][0]
        assert lone["seed"] == 4 and lone["planner_label"] == "vlm" \
            and lone["missing_side"] == "flat_mock", lone
        assert not state["duplicates"], state["duplicates"]
        assert not state["other_planners"], state["other_planners"]
        assert not state["bad_epsilon"], state["bad_epsilon"]

        by_key = dict(((p["scenario_id"], p["perturbation_id"], p["seed"]), p)
                      for p in state["pairs"])
        p1 = by_key[("s1_far_target", "p_none", 1)]
        p2 = by_key[("s1_far_target", "p_none", 2)]
        p3 = by_key[("s1_far_target", "p_none", 3)]
        assert abs(p1["epsilon"] - 0.25) < 1e-9, p1["epsilon"]
        assert abs(p1["delta_progress"] - 0.75) < 1e-9, p1
        assert p1["verdict"] == "benefit", p1
        assert p1["delta_success"] == 1, p1
        assert abs(p2["delta_progress"] - 0.20) < 1e-9, p2
        assert p2["verdict"] == "neutral", "delta 0.20 < epsilon 0.25 must be neutral: %r" % p2
        assert abs(p3["delta_progress"] + 0.50) < 1e-9, p3
        assert p3["verdict"] == "harm", p3
        say("[self-test] dead-band works: delta=+0.75 benefit, +0.20 neutral (eps=0.25), "
            "-0.50 harm")

        group = None
        for row in state["paired_groups"]:
            if row["scenario_id"] == "s1_far_target" and row["perturbation_id"] == "p_none":
                group = row
        assert group is not None, "missing group summary"
        assert group["n_pairs"] == 3, group
        assert group["benefit"] == 1 and group["neutral"] == 1 and group["harm"] == 1, group
        assert abs(group["mean_delta_progress"] - 0.15) < 1e-9, group["mean_delta_progress"]
        assert abs(group["mean_delta_success"] - (1.0 / 3.0)) < 1e-9, group["mean_delta_success"]
        assert abs(group["benefit_rate"] - (1.0 / 3.0)) < 1e-9, group
        say("[self-test] group s1_far_target/p_none: n=3 mean_delta_progress=%.4f "
            "benefit/neutral/harm=1/1/1 mean_delta_success=%.4f"
            % (group["mean_delta_progress"], group["mean_delta_success"]))

        overall = state["paired_overall"]
        # 3 (scenario A) + 1 (B/p_none, delta 0) + 1 (B/p_cam, delta 0) + 1 (scenario C,
        # delta -0.5 == exactly one of its two subgoals) = 6 pairs.
        # Scenario C counts as HARM, not neutral: losing exactly one whole subgoal is the
        # smallest loss the metric can express, so the dead-band must not swallow it. See
        # verdict_for -- this expectation changed together with that rule.
        assert overall["n_pairs"] == 6, overall
        assert overall["benefit"] == 1 and overall["harm"] == 2 and overall["neutral"] == 3, overall
        expected_mean = (0.75 + 0.20 - 0.50 + 0.0 + 0.0 - 0.5) / 6.0
        assert abs(overall["mean_delta_progress"] - expected_mean) < 1e-9, overall
        pc = by_key[("s7_setup_probe", "p_none", 1)]
        assert abs(pc["epsilon"] - 0.5) < 1e-9 and pc["verdict"] == "harm", \
            "delta -0.5 with epsilon 0.5 is exactly one lost subgoal -> harm: %r" % pc
        # Rounding guard: the runner stores ordered_progress to 4 dp, so a one-subgoal gain on
        # 3 subgoals arrives as 0.3333 against an epsilon of 0.333333... It must still read as
        # benefit -- that regression is what motivated comparing in subgoal units.
        assert verdict_for(0.3333, 1.0 / 3.0) == "benefit", "4-dp rounding must not hide a gain"
        assert verdict_for(-0.3333, 1.0 / 3.0) == "harm", "4-dp rounding must not hide a loss"
        assert verdict_for(0.30, 1.0 / 3.0) == "neutral", "a sub-subgoal delta stays neutral"
        say("[self-test] overall: n_pairs=6 benefit/neutral/harm=1/3/2 mean_delta_progress=%.4f"
            % overall["mean_delta_progress"])

        # ---- B: aggregates ---------------------------------------------------------------------
        agg = dict(((r["scenario_id"], r["perturbation_id"], r["planner_label"]), r)
                   for r in state["aggregates"])
        row = agg[("s1_far_target", "p_none", "vlm")]
        assert row["n"] == 4, row
        assert abs(row["success_rate"] - 0.25) < 1e-9, row
        assert abs(row["mean_ordered_progress"] - (1.00 + 0.70 + 0.25 + 0.50) / 4.0) < 1e-9, row
        say("[self-test] aggregates ok: s1/p_none/vlm n=4 success_rate=%.2f mean_ordered=%.4f"
            % (row["success_rate"], row["mean_ordered_progress"]))

        # ---- D: sensitivity ----------------------------------------------------------------------
        sens = dict(((r["planner_label"], r["perturbation_id"]), r) for r in state["sensitivity"])
        cam = sens[("vlm", "p_cam_smudge")]
        assert cam["n_common_scenarios"] == 1, cam
        assert abs(cam["delta_vs_p_none"] + 0.75) < 1e-9, cam
        assert abs(sens[("vlm", "p_none")]["delta_vs_p_none"]) < 1e-9, sens[("vlm", "p_none")]
        say("[self-test] sensitivity ok: vlm under p_cam_smudge loses %.2f ordered_progress "
            "vs p_none on the shared scenario" % cam["delta_vs_p_none"])

        # ---- E: attribution ------------------------------------------------------------------
        buckets = dict((e["episode_key"], e["attribution"]) for e in state["episodes"])
        expected = {
            "s1_far_target__p_none__flat_mock__seed1": "grounding",
            "s1_far_target__p_none__vlm__seed1": "",
            "s1_far_target__p_none__flat_mock__seed2": "execution",
            "s1_far_target__p_none__vlm__seed2": "timeout",
            "s1_far_target__p_none__flat_mock__seed3": "timeout",
            "s1_far_target__p_none__vlm__seed3": "planner",
            "s1_far_target__p_none__vlm__seed4": "timeout",
            "s5_dark_kitchen__p_cam_smudge__flat_mock__seed1": "perception",
            "s5_dark_kitchen__p_cam_smudge__vlm__seed1": "perception",
            "s7_setup_probe__p_none__vlm__seed1": "setup",
            "s7_setup_probe__p_none__flat_mock__seed1": "catastrophic",
        }
        for key in sorted(expected):
            assert buckets[key] == expected[key], \
                "%s -> %r, expected %r" % (key, buckets[key], expected[key])
        counts = state["attribution_counts"]
        assert counts["unattributed"] == 0, "residual bucket must be empty: %r" % counts
        assert sum(counts.values()) == 10, counts
        assert counts["grounding"] == 1 and counts["planner"] == 1 and counts["execution"] == 1
        assert counts["perception"] == 2 and counts["timeout"] == 3
        assert counts["catastrophic"] == 1 and counts["setup"] == 1
        say("[self-test] attribution ok: " + ", ".join(
            "%s=%d" % (b, counts[b]) for b in ATTRIBUTION_ORDER if counts[b]))

        # ---- outputs and determinism -----------------------------------------------------------
        expected_files = ["aggregates.csv", "attribution.csv", "episodes.csv", "episodes.json",
                          "paired_deltas.csv", "paired_summary.csv", "report.md", "sensitivity.csv",
                          "summary.json", "unpaired.csv"]
        actual = sorted(os.listdir(out))
        assert actual == expected_files, "%r != %r" % (actual, expected_files)
        with open(os.path.join(out, "report.md"), "r", encoding="utf-8") as handle:
            report = handle.read()
        for needle in ["# Диагностический отчёт", "## 5. Парные дельты", "## 7. Атрибуция отказов",
                       "## 8. Что это значит", "Приложение A"]:
            assert needle in report, "report is missing %r" % needle
        widest = max(len(line) for line in report.split("\n") if line.startswith("|"))
        assert widest <= MD_MAX_WIDTH, "markdown table is %d columns wide" % widest
        say("[self-test] wrote %d files, widest markdown table line = %d columns"
            % (len(actual), widest))

        out2 = os.path.join(tmp, "report2")
        args2 = argparse.Namespace(**vars(args))
        args2.out = out2
        run_pipeline(args2)
        for name in ["episodes.csv", "aggregates.csv", "paired_deltas.csv", "sensitivity.csv",
                     "attribution.csv", "summary.json"]:
            with open(os.path.join(out, name), "rb") as h1:
                first = h1.read()
            with open(os.path.join(out2, name), "rb") as h2:
                second = h2.read()
            assert first == second, "%s is not deterministic" % name
        say("[self-test] deterministic: a second run produces byte-identical tables")

        # ---- strict mode ------------------------------------------------------------------------
        args3 = argparse.Namespace(**vars(args))
        args3.out = os.path.join(tmp, "report3")
        args3.strict = True
        _, code3 = run_pipeline(args3)
        assert code3 == 2, "strict mode must fail on an unpaired episode, got %d" % code3
        args4 = argparse.Namespace(**vars(args3))
        args4.out = os.path.join(tmp, "report4")
        args4.scenarios = ["s5_dark_kitchen"]
        state4, code4 = run_pipeline(args4)
        assert code4 == 0 and len(state4["episodes"]) == 4, (code4, len(state4["episodes"]))
        say("[self-test] --strict exits 2 on the unpaired episode; --scenario filter narrows to 4")

        say("SELF-TEST PASSED")
        return 0
    except AssertionError as exc:
        print("SELF-TEST FAILED: %s" % exc, file=sys.stderr)
        return 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# --------------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------------

def _flatten(values):
    """--scenario a b --scenario c,d  ->  ['a', 'b', 'c', 'd']"""
    out = []
    for group in values or []:
        for item in (group if isinstance(group, list) else [group]):
            for piece in str(item).split(","):
                piece = piece.strip()
                if piece:
                    out.append(piece)
    return sorted(set(out))


def build_parser():
    parser = argparse.ArgumentParser(
        prog="house_benchmark_report.py",
        description="Aggregate house_scenario_runner episode records into the diagnostic report.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Example:\n"
               "  house_benchmark_report.py --in ~/ros2_ws/house_benchmark \\\n"
               "      --baseline flat_mock --treatment vlm --format md,csv,json\n")
    parser.add_argument("--in", dest="in_dir",
                        help="directory with *.json episode records (searched recursively)")
    parser.add_argument("--out", dest="out", default=None,
                        help="output directory (default: <in>/report)")
    parser.add_argument("--format", dest="format", default="md,csv,json",
                        help="comma-separated subset of md,csv,json (default: all three)")
    parser.add_argument("--baseline", default="flat_mock",
                        help="planner_label used as the paired baseline (default: flat_mock)")
    parser.add_argument("--treatment", default="vlm",
                        help="planner_label used as the paired treatment (default: vlm)")
    parser.add_argument("--scenario", dest="scenario", nargs="+", action="append",
                        help="keep only these scenario_id values (repeatable, comma-separated ok)")
    parser.add_argument("--perturbation", dest="perturbation", nargs="+", action="append",
                        help="keep only these perturbation_id values (repeatable)")
    parser.add_argument("--perturbation-dir", dest="perturbation_dir", default=None,
                        help="optional config/scenarios/perturbations dir; when PyYAML is "
                             "available the overlays decide which perturbations touch the camera")
    parser.add_argument("--strict", action="store_true",
                        help="exit non-zero if any episode has no paired partner")
    parser.add_argument("--self-test", dest="self_test", action="store_true",
                        help="fabricate synthetic records, run the pipeline, assert the metrics")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.self_test:
        return _self_test()

    if not args.in_dir:
        parser.error("--in is required (or use --self-test)")
    in_dir = os.path.abspath(os.path.expanduser(args.in_dir))
    if not os.path.isdir(in_dir):
        print("ERROR: --in %s is not a directory" % in_dir, file=sys.stderr)
        return 1

    formats = [f.strip().lower() for f in str(args.format).split(",") if f.strip()]
    unknown = [f for f in formats if f not in ("md", "csv", "json")]
    if unknown:
        parser.error("unknown --format value(s): %s" % ", ".join(unknown))
    if not formats:
        parser.error("--format must name at least one of md,csv,json")

    args.formats = formats
    args.scenarios = _flatten(args.scenario)
    args.perturbations = _flatten(args.perturbation)

    state, code = run_pipeline(args)

    print("episodes: %d   skipped files: %d   pairs: %d   unpaired: %d"
          % (len(state["episodes"]), len(state["skipped"]), len(state["pairs"]),
             len(state["unpaired"])))
    overall = state["paired_overall"]
    if overall["n_pairs"]:
        print("paired %s -> %s: mean delta_progress %s, benefit/neutral/harm %s/%s/%s"
              % (state["baseline"], state["treatment"], _fmt(overall["mean_delta_progress"]),
                 _pct(overall["benefit_rate"]), _pct(overall["neutral_rate"]),
                 _pct(overall["harm_rate"])))
    counts = state["attribution_counts"]
    named = ", ".join("%s=%d" % (b, counts[b]) for b in ATTRIBUTION_ORDER if counts[b])
    print("failure attribution: %s" % (named or "нет неуспешных эпизодов"))
    for path in state["written"]:
        print("wrote %s" % path)
    if code:
        print("STRICT: %d unpaired episode(s) - see unpaired.csv" % len(state["unpaired"]),
              file=sys.stderr)
    return code


if __name__ == "__main__":
    sys.exit(main())
