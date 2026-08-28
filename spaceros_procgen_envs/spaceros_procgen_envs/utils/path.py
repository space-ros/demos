from os import listdir, path
from typing import List


def abs_listdir(dir: str) -> List[str]:
    return [path.realpath(path.join(dir, script)) for script in listdir(dir)]
