---
title: Git
category: Ubuntu
layout: 2017/sheet
tags: [Tools]
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

## gitの操作ログを見る

```bash
git reflog

$ git reflog
7983f3ba (HEAD -> support-tracker-protocol, origin/support-tracker-protocol) HEAD@{0}: commit: ノード構成図をアップデート
153022a0 HEAD@{1}: commit: 使用しなくなったメッセージをコメントアウト
79a70a41 HEAD@{2}: commit: vision_nodeを削除
382f6133 HEAD@{3}: commit: 使わないdocker構成を削除
a486b6b0 HEAD@{4}: commit: 不要なincludeを削除
a4f357c5 HEAD@{6}: commit: world_model_publisherの設定
6514d6fd HEAD@{7}: commit: robocup_ssl_commからtracker関連を削除
60abc7c0 HEAD@{9}: commit: シナリオ用docker-composeファイルにautorefを追加
1c76934c HEAD@{10}: checkout: moving from test-planner to support-tracker-protocol
28e952fa (test-planner) HEAD@{11}: cherry-pick: テストを四角に
6f7a22f3 HEAD@{12}: cherry-pick: テスト用プランナー
cd015fe0 HEAD@{14}: checkout: moving from adjust to test-planner
ac9a799b (adjust) HEAD@{15}: cherry-pick: 調整
```
