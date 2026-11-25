"""Select a process-local RKNN runtime without replacing system libraries."""
import os
from pathlib import Path


def rknn_lite_class():
    from rknnlite.api import RKNNLite
    import rknnlite.api.rknn_lite as lite_module

    value = os.environ.get("RKNN_RUNTIME_LIB")
    if value:
        library = Path(value).expanduser().resolve(strict=True)
        if not library.is_file() or library.name != "librknnrt.so":
            raise ValueError("RKNN_RUNTIME_LIB must name an existing librknnrt.so")
        # Lite2 2.3.2 chooses /usr/lib/librknnrt.so by an absolute path on
        # aarch64. LD_LIBRARY_PATH cannot override that choice. Override only
        # the SDK's path selection; inference and numerical settings are intact.
        base = getattr(lite_module, "_deployment_runtime_base", lite_module.RKNNRuntime)
        lite_module._deployment_runtime_base = base

        class ExplicitLibraryRuntime(base):
            def _get_rknn_api_lib_path(self):
                return str(library)

        lite_module.RKNNRuntime = ExplicitLibraryRuntime
    return RKNNLite
