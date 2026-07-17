from powertrain_ros.preflight import PreflightResult, check_preflight


def _env(stop_mm="200", provenance="COMMISSIONED"):
    return {
        "STOP_MM": stop_mm,
        "STOP_MM_PROVENANCE": provenance,
    }


def test_commissioned_preflight_is_ok():
    result = check_preflight(_env(), lambda: {"ops_console.token"})

    assert result == PreflightResult(
        ok=True,
        failures=(),
        warnings=(),
        stop_mm=200.0,
        provenance="COMMISSIONED",
    )


def test_bench_preflight_is_ok_with_warning():
    result = check_preflight(
        _env(provenance="BENCH"),
        lambda: {"ops_controller.token"},
    )

    assert result.ok is True
    assert result.failures == ()
    assert any("BENCH" in warning for warning in result.warnings)
    assert result.provenance == "BENCH"


def test_missing_stop_mm_is_a_failure():
    env = _env()
    del env["STOP_MM"]

    result = check_preflight(env, lambda: {"ops_console.token"})

    assert result.ok is False
    assert any("STOP_MM" in failure for failure in result.failures)
    assert result.stop_mm is None


def test_non_numeric_stop_mm_is_a_failure():
    result = check_preflight(
        _env(stop_mm="not-a-number"),
        lambda: {"ops_console.token"},
    )

    assert result.ok is False
    assert any("STOP_MM" in failure for failure in result.failures)
    assert result.stop_mm is None


def test_out_of_range_stop_mm_is_a_failure():
    for stop_mm in ("49.9", "2000.1"):
        result = check_preflight(
            _env(stop_mm=stop_mm),
            lambda: {"ops_console.token"},
        )

        assert result.ok is False
        assert any("STOP_MM" in failure for failure in result.failures)


def test_missing_or_invalid_provenance_is_a_failure():
    missing_env = _env()
    del missing_env["STOP_MM_PROVENANCE"]

    for env in (missing_env, _env(provenance="UNKNOWN")):
        result = check_preflight(env, lambda: {"ops_console.token"})

        assert result.ok is False
        assert any("STOP_MM_PROVENANCE" in failure for failure in result.failures)
        assert result.provenance is None


def test_no_ops_token_is_a_failure():
    result = check_preflight(_env(), lambda: set())

    assert result.ok is False
    assert any("token" in failure.lower() for failure in result.failures)
