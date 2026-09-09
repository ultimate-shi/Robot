"""使用方法：pytest 运行本文件，验证网页 SegFormer 叠加接口的静态合同。"""

from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def test_web_exposes_optional_segmentation_overlay():
    server = (PACKAGE / 'robot_brain' / 'web_server.py').read_text(
        encoding='utf-8')
    html = (PACKAGE / 'web' / 'index.html').read_text(encoding='utf-8')
    javascript = (PACKAGE / 'web' / 'app.js').read_text(encoding='utf-8')
    assert "@app.get('/api/segmentation.jpg')" in server
    assert 'id="segmentationToggle"' in html
    assert 'id="segmentationLegend"' in html
    assert "? '/api/segmentation.jpg' : '/api/frame.jpg'" in javascript
    assert 'drawSegmentationLegend' in javascript
    assert "response.status === 204" in javascript
