from pathlib import Path

project = Path(SPECPATH)
workspace = project.parent
a = Analysis(
    [str(project / "desktop_entry.py")],
    pathex=[str(project / "python_app"), str(workspace)],
    binaries=[],
    datas=[(str(project / "assets" / "isu_xr_lab_logo.png"), "assets"),
           (str(project / "assets" / "isu_xr_lab_header.png"), "assets")],
    hiddenimports=[],
    hookspath=[], hooksconfig={}, runtime_hooks=[], excludes=[], noarchive=False,
)
pyz = PYZ(a.pure)
exe = EXE(pyz, a.scripts, [], exclude_binaries=True,
          name="ISU-XR-OpenManipulator-Smooth", debug=False, bootloader_ignore_signals=False,
          strip=False, upx=False, console=False)
coll = COLLECT(exe, a.binaries, a.datas, strip=False, upx=False,
               name="ISU-XR-OpenManipulator-Smooth")
