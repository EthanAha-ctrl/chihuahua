import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

spec = importlib.util.spec_from_file_location('bootstrap', Path(__file__).with_name('bootstrap.py'))
bootstrap = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bootstrap)


class BootstrapIntegrityTests(unittest.TestCase):
    def test_corrupt_cached_archive_fails_without_downloading(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'downloads').mkdir()
            (root / 'downloads/tool.zip').write_bytes(b'corrupt')
            with patch.object(bootstrap, 'TOOLS', root), patch.object(bootstrap.urllib.request, 'urlopen') as download:
                with self.assertRaisesRegex(RuntimeError, 'SHA256 mismatch'):
                    bootstrap.install('tool', {'archive': 'tool.zip', 'sha256': '0' * 64})
                download.assert_not_called()

    def test_zip_path_escape_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'downloads').mkdir()
            archive = root / 'downloads/tool.zip'
            with zipfile.ZipFile(archive, 'w') as package:
                package.writestr('../escaped', b'bad')
            with patch.object(bootstrap, 'TOOLS', root):
                with self.assertRaisesRegex(RuntimeError, 'Unsafe archive member'):
                    bootstrap.install('tool', {'archive': 'tool.zip', 'sha256': bootstrap.digest(archive), 'format': 'zip'})
            self.assertFalse((root / 'escaped').exists())

    def test_unsupported_platform_fails_before_install(self):
        with patch.object(bootstrap.platform, 'system', return_value='Darwin'), patch.object(bootstrap, 'install') as install:
            with self.assertRaisesRegex(RuntimeError, 'Linux x86_64 only'):
                bootstrap.main()
            install.assert_not_called()
