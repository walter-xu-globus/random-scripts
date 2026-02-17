# -*- mode: python ; coding: utf-8 -*-

a = Analysis(
    ['main.py'],
    pathex=[],
    binaries=[
        ('gps_ik_python_bindings_old.cp312-win_amd64.pyd', '.'),
        ('MSVCP140D.dll', '.'),      # Must be in same dir as .pyd
        ('VCRUNTIME140D.dll', '.'),
        ('ucrtbased.dll', '.'),
        ('Qt5Cored.dll', '.'),
    ],
    datas=[
        ('DHConfig.ini', '.'),
        ('config.ini', '.'),
    ],
    hiddenimports=[
        'PyQt6.QtCore',
        'PyQt6.QtWidgets',
        'PyQt6.QtGui',
        'matplotlib.backends.backend_qt5agg',
        'numpy',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],  # ← IMPORTANT: Empty list here
    name='GPS_IK_Simulator',
    debug=True,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=None,
)

# ✅ Add COLLECT to create a folder with all DLLs (not single-file exe)
coll = COLLECT(
    exe,
    a.binaries,  # ← This includes all the DLLs
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='GPS_IK_Simulator',
)