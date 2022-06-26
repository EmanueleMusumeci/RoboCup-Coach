# -*- coding: utf-8 -*-
#
# Copyright 2021 Francesco Fuggitti, Marco Favorito
#
# ------------------------------
#
# This file is part of planning-with-past.
#
# planning-with-past is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# planning-with-past is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with planning-with-past.  If not, see <https://www.gnu.org/licenses/>.
#
# pylint: disable-all

"""Wrapper to MyND."""
import os
import sys
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Dict, Optional
import time


import networkx
import pydot

from robocup_spl_temporal_goals import MYND_DIR, MYND_JAR, DEFAULT_ALGORITHM, DEFAULT_HEURISTIC,MYND_POLICY_OUTPUT_FILENAME
from robocup_spl_temporal_goals.third_party.planning_with_past.helpers.utils import cd
from robocup_spl_temporal_goals.third_party.planning_with_past.planners.base import Planner
from robocup_spl_temporal_goals.third_party.planning_with_past.plans import BasePlan, Plan

from robocup_spl_temporal_goals.third_party.planning_with_past.utils.subprocess import launch


class MyNDPlanner(Planner):
    """Wrapper to MyND planner."""

    def __init__(
        self,
        jar_path: Path = MYND_JAR,
        search: str = DEFAULT_ALGORITHM,
        heuristic: str = DEFAULT_HEURISTIC,
        timeout: float = 10.0,
    ):
        """
        Initialize.

        :param jar_path: path to the executable.
        :param search: -search argument.
        :param heuristic: -heuristic argument.
        :param timeout: timeout of execution.
        """
        assert jar_path.exists()
        self._jar_path = jar_path

        self._search = search
        self._heuristic = heuristic
        self.timeout = timeout

    @property
    def jar_path(self) -> Path:
        """Return path to the jar."""
        return self._jar_path

    @property
    def search(self) -> str:
        """Return the search argument."""
        return self._search

    @property
    def heuristic(self) -> str:
        """Return the heuristic argument."""
        return self._heuristic

    def plan(self, domain_path: Path, problem_path: Path, working_dir : Optional[Path] = None, output_log_file_name : str = None, output_policy_file_name : str = None, time_benchmark_mode = False) -> Plan:
        """Planning for temporally extended goals (LTLf or PLTLf)."""
        domain_path = str(Path(domain_path).absolute())
        problem_path = str(Path(problem_path).absolute())
        if working_dir is not None:
            working_dir.mkdir(exist_ok=True)
        else:
            working_dir = Path(tempfile.mkdtemp())
        
        if output_policy_file_name is not None:
            output_path = working_dir / output_policy_file_name
        else:
            output_path = working_dir / MYND_POLICY_OUTPUT_FILENAME

        translate_command = [sys.executable, f"{MYND_DIR}/translator-fond/translate.py", f"{domain_path}",
                            f"{problem_path}"]
        launch(translate_command, cwd=working_dir)
        
        if output_log_file_name is not None:
            output_sas = (working_dir / Path(output_log_file_name+".sas"))
        else:
            output_sas = (working_dir / Path("output.sas")).absolute()
        
        planner_command = ['java',
                        "-Xmx4g",
                        '-jar',
                        str(self.jar_path),
                        str(output_sas),
                        '-search',
                        self._search,
                        '-heuristic',
                        self._heuristic,
                        "-exportDot",
                        str(output_path)
                        ]

        print("Start planner")
        if time_benchmark_mode:
            start = time.time()

        launch(planner_command, cwd=str(working_dir))
        
        if time_benchmark_mode:
            end = time.time()
            return end - start
        else:
            return from_dot_to_policy(policy_dot_path = output_path)


def from_dot_to_policy(policy_dot_path: Path) -> Plan:
    """Transform a DOT policy file into a Plan object."""
    return networkx.drawing.nx_pydot.read_dot(policy_dot_path)
