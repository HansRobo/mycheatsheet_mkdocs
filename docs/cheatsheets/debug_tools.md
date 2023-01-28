---
title: デバッグのための道具
category: Ubuntu
layout: 2017/sheet
tags: [Tools]
updated: 2022-06-16
weight: -10
intro: Debug Tools
---



### ldd

共有ライブラリへの依存関係を表示する

```bash
ldd <options> <executable>
```

出力例

```bash
ldd a.out
 linux-vdso.so.1 =>  (0x00007fff089ff000)
 libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007fe74edb3000)
 /lib64/ld-linux-x86-64.so.2 (0x00007fe74f145000)
```

よく使うオプション

| オプション | 説明               |
|-------|------------------|
| `-d`  | 足りないオブジェクトを表示    |
| `-r`  | 足りないオブジェクトと関数を表示 |
| `-u`  | 未使用の依存関係を表示      |
| `-v`  | 全ての情報を表示         |
