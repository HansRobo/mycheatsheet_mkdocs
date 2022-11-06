---
title: KDE
category: Ubuntu
layout: 2017/sheet
tags: [Featured]
updated: 2022-06-16
weight: -10
intro: KDE
---

{: .-three-column}

### KDEの残骸を消す

```bash
sudo apt remove plasma-desktop --autoremove
sudo apt-get remove kde* --autoremove
sudo apt-get remove plasma* --autoremove
sudo update-alternatives --config default.plymouth
sudo update-initramfs -u
sudo update-grub
sudo systemctl disable sddm
sudo systemctl stop sddm
sudo apt-get purge --auto-remove sddm
sudo systemctl enable gdm3
sudo systemctl start gdm3
reboot
```

[参考](https://trendoceans.com/how-to-remove-the-kde-plasma-environment-in-ubuntu/)

### KDEのWalletの無効化

KDE付属のパスワード管理ソフト「kwallet」

~/.config/kwalletrcに

```jsx
Enabled=false
```

と書き込む

（有効化はログアウトかプラズマの再起動が必要）

参考：

[KDEウォレットを無効にする方法は？](https://www.webdevqa.jp.net/ja/kde5/kde%E3%82%A6%E3%82%A9%E3%83%AC%E3%83%83%E3%83%88%E3%82%92%E7%84%A1%E5%8A%B9%E3%81%AB%E3%81%99%E3%82%8B%E6%96%B9%E6%B3%95%E3%81%AF%EF%BC%9F/957133052/)
