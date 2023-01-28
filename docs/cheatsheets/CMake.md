---
title: CMake
category: Ubuntu
layout: 2017/sheet
tags: [Featured]
updated: 2023-01-04
weight: -10
intro: CMake
---



### パッケージの探索結果で分岐する

`find_package(pkg_name)`の結果は`pkg_name_FOUND`に出力されるのを利用する.
REQUIRED指定すると見つからなかった場合にビルドエラーになっていしまうので注意

```CMakeLists.txt
find_package(Boost)
if( Boost_FOUND )
  # do something
endif()
```



### cmakeの関数の引数

#### 基礎

- 名前をつけた引数は`${引数名}`の形で関数内で参照可能
- ↑からはみ出た引数は`${ARGN}`にセミコロン区切りで詰め込まれる

```cmake
function(関数名 引数名1 引数名2)
	message("ARG1 = ${引数名1}")
	message("ARG2 = ${引数名2}")
	message("ARGN = ${ARGN}") 
endfunction()
```

#### パース

```cmake　　　　　　　　　　
cmake_parse_arguments(
	MYFUNC 　　　       # 変数のプレフィックス
	"BOOL_VAR"         # 真偽値をとる変数の名前
	"VAR1;VAR2"        # 値を1つだけとる変数の名前
	"MULTIPLE_OUTPUT"  # 値を複数とる変数の名前
	${ARGN}　　　　　　　# パース元
)
```

- 変数はセミコロン区切りで複数指定可能
- CMakeには名前空間がないのでプレフィックスをつけることで名前の衝突を回避
	- 今回は`MYFUNC_VAR1`とかになる
- パースされなかったものは`PREFIX_UNPARSED_ARGUMENTS`に詰められる
