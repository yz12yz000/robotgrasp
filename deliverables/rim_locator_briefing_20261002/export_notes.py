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
H=Path(__file__).resolve().parent
notes=json.loads((H/'speaker_notes_zh.json').read_text())
pdfmetrics.registerFont(UnicodeCIDFont('STSong-Light'))
w,h=A4
c=canvas.Canvas(str(H/'rim_locator_slides_with_Chinese_notes.pdf'),pagesize=A4)
c.setTitle('Rim Locator — English Slides with Chinese Speaker Notes')
style=ParagraphStyle('notes',fontName='STSong-Light',fontSize=10.5,leading=16,textColor=HexColor('#15283C'),wordWrap='CJK',spaceAfter=9)
for n in notes:
    c.setFillColor(HexColor('#087F83'));c.rect(0,h-8,w,8,fill=1,stroke=0)
    c.setFont('STSong-Light',14);c.drawString(34,h-39,f"第 {n['slide']} 页｜{n['title']}")
    image_width=w-68;image_height=image_width*7.5/(40/3)
    c.drawImage(str(H/f"preview/slide-{n['slide']}.png"),34,h-58-image_height,width=image_width,height=image_height)
    y=h-80-image_height
    c.setFillColor(HexColor('#087F83'));c.setFont('STSong-Light',12);c.drawString(34,y,'中文演讲备注')
    y-=22
    for block in n['notes'].split('\n\n'):
        p=Paragraph(escape(block),style);pw,ph=p.wrap(w-68,y-48)
        assert ph < y-48, ('notes overflow',n['slide'])
        p.drawOn(c,34,y-ph);y-=ph+10
    c.setFillColor(HexColor('#536678'));c.setFont('Helvetica',8)
    c.drawString(34,30,'Source IDs: '+', '.join(n['sources'])+'  |  See evidence_sources.json')
    c.drawRightString(w-34,30,f"{n['slide']} / 6")
    c.showPage()
c.save()
print('Created Chinese notes PDF (6 pages)')
