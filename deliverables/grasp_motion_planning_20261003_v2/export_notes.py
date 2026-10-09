from pathlib import Path
import json
from xml.sax.saxutils import escape
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import Paragraph
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.colors import HexColor

HERE = Path(__file__).resolve().parent
notes = json.loads((HERE/'speaker_notes_zh.json').read_text())
pdfmetrics.registerFont(UnicodeCIDFont('STSong-Light'))
w, h = A4
c = canvas.Canvas(str(HERE/'grasp_motion_planning_2slides_with_Chinese_notes.pdf'), pagesize=A4)
c.setTitle('Grasp Motion Planning — English Slides and Chinese Explanations')
style = ParagraphStyle('notes', fontName='STSong-Light', fontSize=10.5, leading=16,
                       textColor=HexColor('#15283C'), wordWrap='CJK')
for n in notes:
    c.setFillColor(HexColor('#087F83')); c.rect(0, h-8, w, 8, fill=1, stroke=0)
    c.setFont('STSong-Light', 14); c.drawString(34, h-39, f"第 {n['slide']} 页｜{n['title']}")
    iw = w-68; ih = iw*7.5/(40/3)
    c.drawImage(str(HERE/f"preview/slide-{n['slide']}.png"), 34, h-58-ih, width=iw, height=ih)
    y = h-80-ih
    c.setFont('STSong-Light', 12); c.drawString(34, y, '中文解释备注'); y -= 22
    for block in n['notes'].split('\n\n'):
        p = Paragraph(escape(block), style)
        _, ph = p.wrap(w-68, y-44)
        assert ph < y-44, ('notes overflow', n['slide'])
        p.drawOn(c, 34, y-ph); y -= ph+10
    c.setFont('Helvetica', 8); c.setFillColor(HexColor('#536678'))
    c.drawString(34, 28, 'GRASP MOTION PLANNING / 03 OCT 2026')
    c.drawRightString(w-34, 28, f"{n['slide']} / 2")
    c.showPage()
c.save()
print('Created two-page PDF with Chinese explanations below each slide.')
