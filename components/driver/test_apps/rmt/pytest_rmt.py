# SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0

import pytest
from pytest_embedded import Dut


@pytest.mark.esp32
@pytest.mark.esp32s2
@pytest.mark.esp32c3
@pytest.mark.generic
@pytest.mark.parametrize(
    'config',
    [
        'iram_safe',
        'release',
    ],
    indirect=True,
)
def test_rmt(dut: Dut) -> None:
    dut.expect('Press ENTER to see the list of tests')
    dut.write('*')
    dut.expect_unity_test_output()


@pytest.mark.esp32s3
@pytest.mark.octal_psram
@pytest.mark.parametrize(
    'config',
    [
        'iram_safe',
        'release',
    ],
    indirect=True,
)
def test_rmt_psram(dut: Dut) -> None:
    dut.expect('Press ENTER to see the list of tests')
    dut.write('*')
    dut.expect_unity_test_output()
