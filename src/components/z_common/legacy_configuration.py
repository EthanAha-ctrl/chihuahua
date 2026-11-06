"""Adapt Markdown configuration records for the existing geometry/mass callers.

This is the legacy callers' field/value/type schema, not a general table API.
Markdown syntax is handled by markdown-it-py. Extra columns are ignored here.
"""
from pathlib import Path
import json
from markdown_it import MarkdownIt
from markdown_it.tree import SyntaxTreeNode


def read_configuration(path: Path) -> dict:
    tree = SyntaxTreeNode(MarkdownIt('commonmark').enable('table').parse(Path(path).read_text()))
    result = {}
    for table in tree.walk():
        if table.type != 'table':
            continue
        columns = [c.children[0].content for c in table.children[0].children[0].children]
        if not {'field', 'value', 'type'}.issubset(columns):
            continue
        for section in table.children:
            if section.type != 'tbody':
                continue
            for row in section.children:
                record = dict(zip(columns, [c.children[0].content for c in row.children]))
                key, value, kind = record['field'], record['value'], record['type']
                if kind == 'string':
                    decoded = value.replace('<br>', '\n')
                elif kind in ('number', 'boolean', 'null'):
                    decoded = json.loads(value)
                else:
                    raise ValueError(f'{path}: unsupported legacy value type {kind!r} for {key}')
                target = result
                parts = key.split('.')
                for part in parts[:-1]:
                    target = target.setdefault(part, {})
                if parts[-1] in target:
                    raise ValueError(f'{path}: duplicate configuration field {key}')
                target[parts[-1]] = decoded
    if not result:
        raise ValueError(f'{path}: no legacy configuration records found')
    return result


def write_configuration(path: Path, data: dict) -> None:
    lines = ['| field | value | type |', '| --- | --- | --- |']
    def visit(mapping, prefix=''):
        for key, value in mapping.items():
            field = f'{prefix}.{key}' if prefix else key
            if isinstance(value, dict):
                visit(value, field)
            else:
                kind = 'boolean' if isinstance(value, bool) else 'number' if isinstance(value, (float, int)) else 'null' if value is None else 'string'
                text = value if kind == 'string' else json.dumps(value)
                text = text.replace('|', '\\|').replace('\n', '<br>')
                lines.append(f'| {field} | {text} | {kind} |')
    visit(data)
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')
