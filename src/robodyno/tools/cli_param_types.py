# -*-coding:utf-8 -*-
#
# The MIT License (MIT)
#
# Copyright (c) 2023 Robottime(Beijing) Technology Co., Ltd
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Provides parameter types for the CLI."""

import click


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
