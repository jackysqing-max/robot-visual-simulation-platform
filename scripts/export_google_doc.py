#!/usr/bin/env python3
"""Export the copy-ready Markdown as DOCX using only the Python standard library."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import zipfile

W = 'http://schemas.openxmlformats.org/wordprocessingml/2006/main'
ET.register_namespace('w', W)


def child(parent, tag, text=None, **attributes):
    item = ET.SubElement(parent, '{'+W+'}'+tag,
                         {'{'+W+'}'+k: str(v) for k, v in attributes.items()})
    item.text = text
    return item


def main():
    root = Path(__file__).resolve().parents[1]
    target = root/'docs/google_docs_copy.docx'
    document = ET.Element('{'+W+'}document')
    body = child(document, 'body')
    code = False
    for index, line in enumerate((root/'docs/google_docs_copy.md').read_text().splitlines()):
        if line.startswith('```'):
            code = not code
            continue
        paragraph = child(body, 'p')
        props = child(paragraph, 'pPr')
        style = 'Code' if code else 'Title' if index == 0 else 'Heading1' if re.match(r'^\d+\. ', line) else 'Normal'
        child(props, 'pStyle', val=style)
        if code:
            child(props, 'shd', fill='F3F4F6', val='clear')
        text = child(child(paragraph, 'r'), 't', line)
        text.set('{http://www.w3.org/XML/1998/namespace}space', 'preserve')
    section = child(body, 'sectPr')
    child(section, 'pgSz', w=11906, h=16838)
    child(section, 'pgMar', top=1100, bottom=1100, left=1100, right=1100)
    styles = ET.Element('{'+W+'}styles')
    for name, size, font in [('Normal', 22, 'Arial'), ('Title', 36, 'Arial'),
                              ('Heading1', 27, 'Arial'), ('Code', 17, 'Liberation Mono')]:
        style = child(styles, 'style', type='paragraph', styleId=name)
        child(style, 'name', val=name)
        if name != 'Normal':
            child(style, 'basedOn', val='Normal')
        props = child(style, 'pPr')
        child(props, 'spacing', after=0 if name == 'Code' else 90, line=280, lineRule='auto')
        if name in ('Title', 'Heading1'):
            child(props, 'keepNext')
        run = child(style, 'rPr')
        child(run, 'rFonts', ascii=font, hAnsi=font, eastAsia='Noto Sans CJK SC')
        child(run, 'sz', val=size)
        if name in ('Title', 'Heading1'):
            child(run, 'b')
    types = ('<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
             '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
             '<Default Extension="xml" ContentType="application/xml"/>'
             '<Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>'
             '<Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>'
             '</Types>')
    relation_base = 'http://schemas.openxmlformats.org/officeDocument/2006/relationships/'
    def relationship(kind, part):
        return ('<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
                f'<Relationship Id="rId1" Type="{relation_base}{kind}" Target="{part}"/>'
                '</Relationships>')
    with zipfile.ZipFile(target, 'w', zipfile.ZIP_DEFLATED) as archive:
        archive.writestr('[Content_Types].xml', types)
        archive.writestr('_rels/.rels', relationship('officeDocument', 'word/document.xml'))
        archive.writestr('word/_rels/document.xml.rels', relationship('styles', 'styles.xml'))
        archive.writestr('word/document.xml', ET.tostring(document, encoding='utf-8', xml_declaration=True))
        archive.writestr('word/styles.xml', ET.tostring(styles, encoding='utf-8', xml_declaration=True))
    print(target)


if __name__ == '__main__':
    main()
