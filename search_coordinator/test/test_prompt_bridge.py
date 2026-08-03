from search_coordinator.prompt_bridge import PromptBridge


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data)


class _FakeLogger:
    def info(self, *_args, **_kwargs):
        pass


class _FakeNode:
    def __init__(self):
        self.publisher = _FakePublisher()
        self.logger = _FakeLogger()

    def create_publisher(self, *_args, **_kwargs):
        return self.publisher

    def get_logger(self):
        return self.logger


def test_prompt_bridge_clear_latches_empty_prompt():
    node = _FakeNode()
    bridge = PromptBridge(node)

    bridge.publish('find the drawer cabinet')
    bridge.clear('done')

    assert node.publisher.messages == ['drawer cabinet', '']
    assert bridge.last_prompt == ''
