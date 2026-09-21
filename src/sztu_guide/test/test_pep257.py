# Copyright 2015 Open Source Robotics Foundation, Inc.
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

from ament_pep257.main import main
import pytest


# 本项目的文档字符串通篇是中文，pydocstyle 下面这几条和它天然打架。
# 逐条记下为什么关掉，而不是把整个 pep257 检查一关了事：
#   D400/D415  要求摘要行以英文句号结尾。中文句子收在“。”或“）”上，
#              为了过 lint 在中文后面补一个 ASCII 句点只会更难看。
#   D403       要求首词首字母大写，会把 AimDK 判成应写作 Aimdk。
#   D406/D407/D413
#              要求 numpy 风格的 Returns + 虚线下划线，而本项目统一用
#              Google 风格的 "Returns:"。
DOCSTRING_IGNORE = ['D400', 'D415', 'D403', 'D406', 'D407', 'D413']


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test', '--add-ignore'] + DOCSTRING_IGNORE)
    assert rc == 0, 'Found code style errors / warnings'
