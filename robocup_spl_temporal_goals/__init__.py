import inspect
from pathlib import Path

PACKAGE_ROOT = Path(inspect.getframeinfo(inspect.currentframe()).filename).parent  # type: ignore
REPO_ROOT = PACKAGE_ROOT.parent


MYND_JAR = (PACKAGE_ROOT / "third_party" / "mynd.jar").absolute()
MYND_DIR = (PACKAGE_ROOT / "third_party" / "myND").absolute()
MYND_POLICY_OUTPUT_FILENAME = "policy"
DEFAULT_ALGORITHM = "LAOSTAR"
DEFAULT_HEURISTIC = "FF"