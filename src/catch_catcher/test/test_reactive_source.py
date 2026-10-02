"""v2: in vision-only mode robot B must not see A's plan. A ReactiveCatcher with source=vision has no subscription
to /throw_plan and no /throw/preview client on any of its nodes (the action node and its helper nodes); with
source=plan (benchmark baseline) it subscribes /throw_plan and not /tracker/cube. Own DDS domain; needs the
installed workspace (configs)."""
import os

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("catch_catcher.reactive_node")
from catch_catcher.reactive_node import ReactiveCatcher  # noqa: E402

DOMAIN = 100 + (os.getpid() + 71) % 100


def topics(node):
    subs = {s.topic_name for n in node.all_nodes() for s in n.subscriptions}
    clients = {c.srv_name for n in node.all_nodes() for c in n.clients}
    return subs, clients


@pytest.mark.parametrize("source", ["vision", "plan"])
def test_vision_only_never_sees_the_plan(source):
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=DOMAIN)
    try:
        n = _make(ctx, source)
        subs, clients = topics(n)
        if source == "vision":
            assert "/throw_plan" not in subs, subs
            assert not any("throw" in c for c in clients), clients
            assert "/tracker/cube" in subs
        else:
            assert "/throw_plan" in subs
            assert "/tracker/cube" not in subs
    finally:
        rclpy.shutdown(context=ctx)


def _make(ctx, source):
    import rclpy.node as rn
    orig = rn.Node.__init__

    def init(self, *a, **kw):
        kw.setdefault("context", ctx)
        orig(self, *a, **kw)
    rn.Node.__init__ = init
    try:
        return ReactiveCatcher(source=source)
    finally:
        rn.Node.__init__ = orig
