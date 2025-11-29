#!/usr/bin/env python3
import os
from pathlib import Path
import yaml

def extract_frontmatter(md_file):
    """Markdownファイルからfrontmatterを抽出"""
    with open(md_file, 'r', encoding='utf-8') as f:
        content = f.read()

    if content.startswith('---'):
        try:
            parts = content.split('---', 2)
            if len(parts) >= 3:
                fm = yaml.safe_load(parts[1])
                return fm or {}
        except:
            pass
    return {}

def generate_index():
    docs_dir = Path('docs')
    files = sorted([f for f in docs_dir.glob('*.md') if f.name != 'index.md'])

    # カテゴリー別に分類
    categories = {}
    for file in files:
        fm = extract_frontmatter(file)
        category = fm.get('category') or 'その他'
        title = fm.get('title') or file.stem

        if category not in categories:
            categories[category] = []
        categories[category].append({
            'title': title,
            'file': file.name
        })

    # index.md を生成
    content = [
        "# HansRobo's Cheatsheet",
        "",
        "技術メモとチートシートのコレクション",
        "",
        "## ドキュメント一覧",
        ""
    ]

    for category in sorted(categories.keys()):
        content.append(f"### {category}")
        content.append("")
        for doc in sorted(categories[category], key=lambda x: x['title']):
            content.append(f"- [{doc['title']}]({doc['file']})")
        content.append("")

    # 書き込み
    with open(docs_dir / 'index.md', 'w', encoding='utf-8') as f:
        f.write('\n'.join(content))

    print(f"✅ index.md を生成しました（{len(files)}ファイル）")

if __name__ == '__main__':
    generate_index()
