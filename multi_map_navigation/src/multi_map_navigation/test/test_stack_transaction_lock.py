"""Regression tests for navigation-stack transaction ownership."""

import threading

from multi_map_navigation.stack_transaction_lock import StackTransactionLock


def test_separate_lock_instances_are_mutually_exclusive(monkeypatch, tmp_path):
    monkeypatch.setattr(
        StackTransactionLock,
        'PATH',
        str(tmp_path / 'stack-transaction.lock'),
    )
    owner = StackTransactionLock()
    contender = StackTransactionLock()

    assert owner.acquire()
    try:
        assert contender.acquire() is False
    finally:
        owner.release()

    assert contender.acquire()
    contender.release()


def test_same_thread_can_reenter_without_releasing_global_ownership(
        monkeypatch, tmp_path):
    monkeypatch.setattr(
        StackTransactionLock,
        'PATH',
        str(tmp_path / 'stack-transaction.lock'),
    )
    owner = StackTransactionLock()
    contender = StackTransactionLock()

    assert owner.acquire()
    assert owner.acquire()
    owner.release()
    assert contender.acquire() is False

    owner.release()
    assert contender.acquire()
    contender.release()


def test_bounded_wait_acquires_after_owner_releases(
        monkeypatch, tmp_path):
    monkeypatch.setattr(
        StackTransactionLock,
        'PATH',
        str(tmp_path / 'stack-transaction.lock'),
    )
    owner = StackTransactionLock()
    contender = StackTransactionLock()
    wait_started = threading.Event()
    retry_allowed = threading.Event()
    acquired = []

    assert owner.acquire()

    def controlled_sleep(_delay):
        wait_started.set()
        assert retry_allowed.wait(1.0)

    monkeypatch.setattr(
        'multi_map_navigation.stack_transaction_lock.time.sleep',
        controlled_sleep,
    )

    def acquire_and_release():
        result = contender.acquire(timeout_sec=1.0)
        acquired.append(result)
        if result:
            contender.release()

    worker = threading.Thread(target=acquire_and_release, daemon=True)
    worker.start()
    assert wait_started.wait(0.5)

    owner.release()
    retry_allowed.set()
    worker.join(0.5)

    assert not worker.is_alive()
    assert acquired == [True]


def test_bounded_wait_returns_false_after_timeout(monkeypatch, tmp_path):
    monkeypatch.setattr(
        StackTransactionLock,
        'PATH',
        str(tmp_path / 'stack-transaction.lock'),
    )
    owner = StackTransactionLock()
    contender = StackTransactionLock()
    monotonic_values = iter((10.0, 10.0, 10.2))
    sleeps = []

    assert owner.acquire()
    monkeypatch.setattr(
        'multi_map_navigation.stack_transaction_lock.time.monotonic',
        lambda: next(monotonic_values),
    )
    monkeypatch.setattr(
        'multi_map_navigation.stack_transaction_lock.time.sleep',
        sleeps.append,
    )
    try:
        assert not contender.acquire(timeout_sec=0.1)
    finally:
        owner.release()

    assert sleeps == [0.05]
