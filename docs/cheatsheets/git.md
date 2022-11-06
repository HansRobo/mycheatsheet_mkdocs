---
title: Git
category: Ubuntu
layout: 2017/sheet
tags: [Featured]
updated: 2022-06-16
weight: -10
intro: Git
---



### gitのsubmoduleの移動方法

```bash
git mv <from> <to>
```

### GitHubにssh鍵を追加する

```bash
cd ~/.ssh
ssh-keygen -t rsa
```

- 名前とパスワードを聞かれるので入力
- 公開鍵（〜.pub）の中身をgithubに登録

[https://github.com/settings/keys](https://github.com/settings/keys)

- configに登録

```bash
Host github github.com
  HostName github.com
  IdentityFile ~/.ssh/github
  User git
```

- テスト

```bash
ssh -T github
```
