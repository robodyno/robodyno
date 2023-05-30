# -*-coding:utf-8 -*-
#
# Apache License, Version 2.0
#
# Copyright (c) 2023 Robottime(Beijing) Technology Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Provides parameter types for the CLI."""

import click
from rich.prompt import PromptBase, DefaultType, InvalidResponse
from rich.text import Text


class DeviceIdListParamType(click.ParamType):
    """A click parameter type for device ID list."""

    name = 'device_id_list'

    def convert(self, value, param, ctx):
        if isinstance(value, int):
            return (value,)

        try:
            id_set = set()
            for id_str in value.split(','):
                splited = id_str.split('-')
                if len(splited) == 1:
                    id_set.add(int(splited[0], 0))
                elif len(splited) == 2:
                    id_set.update(range(int(splited[0], 0), int(splited[1], 0) + 1))
                else:
                    raise ValueError
            return tuple(id_set)
        except ValueError:
            self.fail(f'{value!r} is not a valid id list', param, ctx)


ID_LIST = DeviceIdListParamType()


class BasedIntParamType(click.ParamType):
    """A click parameter type for based integer."""

    name = 'integer'

    def convert(self, value, param, ctx):
        if isinstance(value, int):
            return value

        try:
            return int(value, 0)
        except ValueError:
            self.fail(f'{value!r} is not a valid integer', param, ctx)


BASED_INT = BasedIntParamType()


class BasedIntPrompt(PromptBase[int]):
    """A based integer prompt.

    Example:
        >>> id = BasedIntPrompt.ask("Please enter a device id", default=0x01)
    """

    response_type = int
    validate_error_message = '[prompt.invalid]Please enter a valid integer number'

    def render_default(self, default: DefaultType) -> Text:
        """Render the default as (y) or (n) rather than True/False."""
        return Text(f'(0x{default:02X})', style='prompt.default')

    def process_response(self, value: str) -> int:
        """Convert choices to a int."""
        value = value.strip().lower()
        try:
            value = int(value, 0)
        except ValueError as error:
            raise InvalidResponse(self.validate_error_message) from error
        return value
