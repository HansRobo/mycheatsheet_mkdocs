---
title: GitHub Actions
category: Tools
layout: 2017/sheet
tags: [Tools]
updated: 2023-01-28
weight: -10
intro: GitHub Actions
---

## ファイルを同期したい

[Repo File Sync Action · Actions · GitHub Marketplace · GitHub](https://github.com/marketplace/actions/repo-file-sync-action)
これを使う．同期するファイルは`sync.yaml`という設定ファイルで管理．
同期先のパスが変わってもOK

## ファイルの変更監視

[[GitHub Actions]ファイルの差分や更新状態を元にStepの実施を切り分けてみる | DevelopersIO](https://dev.classmethod.jp/articles/switch-step-by-file-conditions/)

## イベントの中身を知りたい

pull_request
[Webhook のイベントとペイロード - GitHub Docs](https://docs.github.com/ja/developers/webhooks-and-events/webhooks/webhook-events-and-payloads#pull_request)


## paths, paths-ignore フィルターについて

- 同時に両方使えない
- pathsでは！が使える
	- paths-ignoreで書けてpathsで書けない表現は無く，逆は成り立たない

参考：[GitHub Actionsにおけるpathsとpaths-ignoreをベン図で理解する - Qiita](https://qiita.com/nacam403/items/3e2a5df5e88ba20aa76a)

### GitHub Actionsの中でsshを使ったgit cloneがしたい

まずは，パーソナルアクセストークンを使った方法で代替できないかどうかを考える．
それでもやりたければ以下の彩都を参考にする

[How to clone a private repository in GitHub Action using SSH | by Samyakt Jain | Medium](https://samyaktjain24.medium.com/how-to-clone-a-private-repository-in-github-action-using-ssh-38d0de8c09d8)