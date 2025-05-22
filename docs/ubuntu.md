---
title: ubuntu
category: Ubuntu
layout: 2017/sheet
tags: [Featured]
updated: 2022-06-16
weight: -10
intro: Ubuntu
---

## `apt-fast`

aptのダウンロード並列化によって爆速化する

```bash
sudo add-apt-repository ppa:apt-fast/stable
sudo apt install apt-fast
```

## `apt-key`の代替手段

廃止となるapt-keyの代替手段について  
[Ubuntu 22.04 その20 - apt-keyの廃止方針と移行措置・Ubuntu 22.04 LTSが最後に - kledgeb](https://kledgeb.blogspot.com/2022/01/ubuntu-2204-20-apt-keyubuntu-2204-lts.html)

### 代替手段1：`/etc/apt/trusted.gpg.d/`に鍵を保存する

該当ディレクトリに保存すると自動で使われるようになる

### 代替手段2：`/usr/local/share/keyrings/`に鍵を保存し，`source.list`で参照する

`/etc/apt/trusted.gpg.d/`を使用する方法は `apt-key`の問題でもあった、追加した鍵がすべてのリポジトリに対して適用されてしまうセキュリティ上の問題が解決されていない。
以下のように`source.list`にて`signed-by`で鍵を指定することで問題は解決される。

```source.list
deb [arch=amd64 signed-by=/usr/local/share/keyrings/google.gpg] http://dl.google.com/linux/chrome/deb/ stable main
```

参考：[apt-keyはなぜ廃止予定となったのか/今後リポジトリ鍵はどう運用すべきか | gihyo.jp](https://gihyo.jp/admin/serial/01/ubuntu-recipe/0675#sec3)

## CapsLockキーをMenuキーにする

`/etc/default/keyboard`を編集して以下の一行を追加する

```/etc/default/keyboard
XKBOPTIONS="menu:nocaps"
```

編集が終了したら再起動

## フォルダを英語化する

以下を実行

```bash
LANG=C xdg-user-dirs-gtk-update
```

## aptのupgrade系コマンドの違い

aptのサブコマンドで似たような以下の3つのコマンドがある

- upgrade
- dist-upgrade
- full-upgrade

### sudo apt upgrade

sources.listで設定されたPPAなどの取得元から利用可能なアップグレードをインストールする。

アップグレードするパッケージに新たな依存関係が追加された場合、新しい依存パッケージが追加でインストールされるが、
**アップグレードの過程で何らかのパッケージの削除が必要となる場合，アップグレードは行われない**

### sudo apt-get dist-upgrade / full-upgrade

**dist-upgradeとfull-upgradeは同様の機能**

dist-upgradeはapt-getとの互換性の為に残されている気がする。

**upgradeとは違い，システム全体をアップグレードするためなら既存パッケージの削除を厭わない．**

## Discordを最小化状態で起動する(Ubuntu)

```bash
discord --start-minimized
```

[Option for Discord to start minimized on Linux](https://support.discord.com/hc/en-us/community/posts/360048037971-Option-for-Discord-to-start-minimized-on-Linux)

## Slackを最小化した状態で立ち上げる

`-u`オプションをつける

```bash
slack -u
```

## シンボリックリンクコマンドの覚え方

なぜか、覚えられない`ln -s`  
実は`cp`や`mv`コマンドとリンク元の登録名の並びが一緒！

```bash
ln -s リンク元 登録名
```

## XML -> YAMLの変換

<!-- cspell:ignore ATOMOS libconfig markitdown  -->

```bash
sudo apt install libxml-compile-perl libconfig-yaml-perl
xml2yaml -x xml.xml -s schema.xsd yaml.yaml
```

## キャッシュクリア色々

### apt

```bash
sudo apt clean
sudo apt --fix-broken install
```

### pip

```bash
rm -rf ~/.cache/pip
```

## ファイルの最大ウォッチ数を増やす

  IDEをたくさん開き過ぎたときなど、OSのファイルウォッチ数が足りない時がある

<!-- cspell:ignore inotify stamemo -->

```bash
sudo echo "fs.inotify.max_user_watches=524288" >> /etc/sysctl.conf
sudo sysctl -p
```

## イコライザを使う

PulseEffectsというのが便利

```bash
sudo apt install lsp-plugins-lv2
sudo apt install pulseeffects
```

エフェクトの中でも応答関数をいじる（？）Convolverというのが良い

[EasyEffects-Presets/Dolby ATMOS ((128K MP3)) 1.Default.irs at master · JackHack96/EasyEffects-Presets · GitHub](<https://github.com/JackHack96/EasyEffects-Presets/blob/master/irs/Dolby%20ATMOS%20((128K%20MP3))>
個人的には↑のDolby ATOMOSを名乗っているプリセットが好み

## mkdocsで箇条書きのインデントが消える

[MkDocs でスペース2個のインデントをリストのネストとして認識させたい場合 - stamemo](https://stakiran.hatenablog.com/entry/2018/08/02/202816)

## AppImageをインストールする

ポータブルでどこでも開けるAppImageだが、とくにUbuntuではユーザーが実行ファイルの場所を管理しないといけないので常用する場合はちょっと面倒である。

そんな時に便利なのが、AppImage Launcher。
これを使えば、他のインストールしたソフトと同じようにSuperキーを押すと出てくる検索画面に出てきたり、タスクバーのお気に入りに登録できるようにしてくれる。
具体的には、AppImageの初回起動時にセットアップ画面が出てくるのでOK連打するだけ。

[Releases · TheAssassin/AppImageLauncher](https://github.com/TheAssassin/AppImageLauncher/releases)

インストール直後に実行するとAppImageのデフォルト格納場所を決めるウィザードが出てくる。
基本的にデフォルト（~/Applications）で問題ない。

<img src="../public_images/Screenshot from 2024-12-12 20-56-51.png" width="50%"/>

以後、AppImageファイルを実行すると以下のような画面が出てくるので、「Integrate and run」を選択すればインストールを行ってくれるようになる。

<img src="../public_images/Screenshot from 2024-12-12 20-57-04.png" width="50%"/>

## MarkItDown

すべてをMarkdownにするツール

### インストール

```bash
pipx install git+https://github.com/microsoft/markitdown.git
```

### YouTube

コマンド例

```bash
markitdown https://youtu.be/nuQvDqgdzMg
```

出力例

```markdown
# YouTube

## Choreonoid ROS2 Demo

### Video Metadata
- **Views:** 57
- **Keywords:** 動画,ビデオ,共有,カメラ付き携帯電話,動画機能付き携帯電話,無料,アップロード,チャンネル,コミュニティ,YouTube,ユーチューブ
- **Runtime:** PT0M28S

### Description
ROSJPのLTで発表したときに使用したデモ動画です。
```


## Davinchi Resolve for Ubuntu 24.04

```bash
sudo apt install libfuse-dev libapr1-dev libxcb-cursor0
```
