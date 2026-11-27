"""Select a plotting font without requiring the report's private font assets."""
from pathlib import Path


def plot_font_family(report_font):
    from matplotlib import font_manager

    report_font = Path(report_font)
    if report_font.is_file():
        font_manager.fontManager.addfont(str(report_font))
        return font_manager.FontProperties(fname=str(report_font)).get_name()
    installed = {font.name for font in font_manager.fontManager.ttflist}
    for family in ('Noto Sans CJK SC', 'Source Han Sans SC', 'WenQuanYi Zen Hei',
                   'Noto Sans CJK JP', 'Microsoft YaHei', 'SimHei', 'PingFang SC',
                   'Heiti SC', 'Songti SC'):
        if family in installed:
            return family
    return 'DejaVu Sans'
