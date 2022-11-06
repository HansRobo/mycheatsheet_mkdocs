---
title: ubuntu
category: Ubuntu
layout: 2017/sheet
tags: [Featured]
updated: 2022-06-16
weight: -10
intro: Ubuntu
---


{: .-three-column}

### CapsLockキーをMenuキーにする

`/etc/default/keyboard`を編集して以下の一行を追加する

```/etc/default/keyboard
XKBOPTIONS="menu:nocaps"`
```

編集が終了したら再起動

### フォルダを英語化する

以下を実行

```bash
LANG=C xdg-user-dirs-gtk-update
```

### aptのupgrade系コマンドの違い

aptのサブコマンドで似たような以下の3つのコマンドがある

- upgrade
- dist-upgrade
- full-upgrade

#### sudo apt upgrade

sources.listで設定されたPPAなどの取得元から利用可能なアップグレードをインストールする．

アップグレードするパッケージに新たな依存関係が追加された場合，新しい依存パッケージが追加でインストールされるが，
**アップグレードの過程で何らかのパッケージの削除が必要となる場合，アップグレードは行われない**

#### sudo apt-get dist-upgrade / full-upgrade

**dist-upgradeとfull-upgradeは同様の機能**

dist-upgradeはapt-getとの互換性の為に残されている気がする．

**upgradeとは違い，システム全体をアップグレードするためなら既存パッケージの削除を厭わない．**

### Discordを最小化状態で起動する(Ubuntu)

```bash
discord --start-minimized
```

[Option for Discord to start minimized on Linux](https://support.discord.com/hc/en-us/community/posts/360048037971-Option-for-Discord-to-start-minimized-on-Linux)



### Slackを最小化した状態で立ち上げる

-uオプションをつける

```bash
slack -u
```

### シンボリックリンクコマンドの覚え方

なぜか，覚えられない`ln -s`  
実は`cp`や`mv`コマンドとリンク元と登録名の並びが一緒！

```bash
 ln -s リンク元 登録名
```

### XML -> YAMLの変換

```bash
sudo apt install libxml-compile-perl libconfig-yaml-perl
xml2yaml -x xml.xml -s schema.xsd yaml.yaml
```

### キャッシュクリア色々

#### apt

```bash
sudo apt clean
sudo apt --fix-broken install
```
#### pip

```bash
rm -rf ~/.cache/pip
```