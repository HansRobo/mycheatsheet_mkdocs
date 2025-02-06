---
title: ansible
category: Tool
layout: 2017/sheet
tags: [Tools]
updated: 2022-06-16
created: 2023-01-28 04:04:29
weight: -10
intro: ansible
---

## インストール方法

[autoware/setup-dev-env.sh at main · autowarefoundation/autoware · GitHub](https://github.com/autowarefoundation/autoware/blob/main/setup-dev-env.sh#L123-L126)

```bash
python3 -m pipx ensurepath
export PATH="${PIPX_BIN_DIR:=$HOME/.local/bin}:$PATH"
pipx install --include-deps --force "ansible==6.*"
```

## yes/no ではなく true/false

[よこち on Twitter: "ansible のドキュメントにおけるにおける boolean は yes/no が遣われがちだったが、ansible-lint では true/false でないとエラーになる。どうしたもんでしょという issue。 https://t.co/dcNl9R83sl" / Twitter](https://twitter.com/akira6592/status/1554971476612501505)

[[Vote ended on 2022-08-03] Disconnect between Docs and Ansible-lint in regards to truthy statements (booleans) · Issue #116 · ansible-community/community-topics · GitHub](https://github.com/ansible-community/community-topics/issues/116)

議論の結果、yes/noはやめてtrue/falseを使おうとなったようで、ドキュメントもtrue/falseで統一されるようになった

## `apt_repository`の書き込み先

`filename`を指定しない場合、`repo`のURLからいい感じに書き込み先が生成される。

例：

- `repo` : `deb https://repo.vivaldi.com/archive/deb/ stable main`
- 生成される書き込み先：`repo_vivaldi_com_archive_deb.list`

ただ、こういったアプリはあとからアップデートなどで別に `vivaldi.list`などが追加され、
`apt update`したときに

```bash
Target Packages (main/binary-amd64/Packages) is configured multiple times in /etc/apt/sources.list.d/packages_microsoft_com_repos_code.list:1 and /etc/apt/sources.list.d/vscode.list:3
```

みたいな警告がうるさいので以下のように `dest`をしっかり設定するほうが良い

```ansible
- name: add ppa to source list
 apt_repository:
  repo: "deb https://repo.vivaldi.com/archive/deb/ stable main"
  state: present
  update_cache: true
  filename: vivaldi
 become: true
```

### 変数名にハイフンを使ってはいけない

以下のようにハイフンを使うとエラーが出る

```ansible
- name: check gnome-terminal
 stat:
  path: /usr/bin/gnome-terminal
 register: gnome-terminal-stat
```

エラー

```shell
fatal: [localhost]: FAILED! => {"msg": "Invalid variable name in 'register' specified: 'gnome-terminal-stat'"}
```
