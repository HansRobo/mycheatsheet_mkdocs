---
title: C++｜cppreference.com勉強日記
category: C++
layout: 2017/sheet
tags: [Featured]
updated: 2022-06-16
weight: -10
intro: C++｜cppreference.com勉強日記
---



### [関数Tryブロック](https://ja.cppreference.com/w/cpp/language/function-try-block)

以下のようなコードがかけちゃうらしい．  
マジか，tryそんなとこに書けたんか

```cpp
int main() try {}
catch (...){}
```

### [main関数](https://ja.cppreference.com/w/cpp/language/main_function)

いわゆるエントリポイント，`main`関数

- `main`関数は`int`型だが，return文はなくてもOK
- `auto main()`という書き方はダメらしい
- `return N`は`std::exit(N)`と同じ意味を持つ
- `main`関数が関数`try`ブロックになっていてもmain関数の外で定義された（しかし，`main`関数の終了と同時に破棄される）静的オブジェクトのデストラクタで投げられた例外はキャッチできない

### [std::min_element](https://ja.cppreference.com/w/cpp/algorithm/min_element), [std::max_element](https://ja.cppreference.com/w/cpp/algorithm/max_element)

- 最低限，探索範囲を[first, last)で指定できる．
- 比較には`<`演算子を使用（`max_element`は`>`演算子を使うわけではない）
- `<`演算子の代わりに比較関数を指定できる．
  - 比較関数は`cmp(new_element, best_element)`という形式で使われる
  - ⇒内容によって意図的に弾きたい場合は，第一引数の内容を参照して弾けば良い（min_elementならfalse,max_elementならtrueを返せば弾ける）
- 実行ポリシーを設定できるらしい（知らんかった)．指定箇所は探索範囲の前．
  - 詳しくは[実行ポリシー](https://ja.cppreference.com/w/cpp/algorithm/execution_policy_tag_t)参照とのこと

### [実行ポリシー(C++17)](https://ja.cppreference.com/w/cpp/algorithm/execution_policy_tag_t)

stlにはたくさんのアルゴリズム関数が存在するが，それらの第一引数にアルゴリズムの処理を実行する上での制約を「実行ポリシー」として与えることができるらしい．
「実行ポリシー」は以下のような種類が存在する

| 実行ポリシー        | 説明                |
|---------------|-------------------|
| `seq`       | 順番に処理を実行する必要がある．当然並列化も出来ない             |
| `par`        | マルチスレッド化による並列化を許可する            |
| `par_unseq` | マルチスレッド化・ベクトル化を許可する          |
| `unseq`       | ベクトル化を許可する（C++20） |

今までのアルゴリズム関数は`seq`で実行されるのが当然だった．
プログラマにから与えられた関数オブジェクトの処理はどういう処理なのかをアルゴリズム関数は知らないのだからそりゃそうだ．
でも，そのプログラマが処理についてアルゴリズム関数にヒントを与えられるのがこの機能というわけだ．
もしかしなくても「OpenMPを使ったfor文の並列化は簡単でいいなぁ」と言っていた時代は終わったのかもしれない

```cpp
std::for_each(std::execution::par, a.begin(), b.end(), [&](int x) {
    std::cout << x << std::endl;
});
```

### [std::minmax_element](https://ja.cppreference.com/w/cpp/algorithm/minmax_element)

- コンテナの範囲を渡すと，最大値と最小値をペアにして返してくれる
- 例によって二項比較関数は`<`演算子相当のものを定義して渡せば良い
- C++17からは実行ポリシーも指定可能
- `std::make_pair(std::min_element(), std::max_element())`よりも効率が良い
- 一番大きい/小さい要素が複数ある場合，↑は最初の要素を返し，本アルゴリズムは最後の要素を返す

```cpp
const auto [min, max] = std::minmax_element(begin(v), end(v));
```

### [std::enable_shared_from_this](https://ja.cppreference.com/w/cpp/memory/enable_shared_from_this)

クラス内で`this`を`shared_ptr`にしたい場合が往々にしてある．（全て自分のコードの場合では設計が悪い場合が多いが，外部ライブラリを使用しているとそうするしか無い場合もありがち）
そういう時に，`std::shared_ptr<T>(this)`などとやってしまうと，`this`は2回以上破棄されてしまいやすい．

そんな時に使うのが，これ`std::enable_shared_from_this`. 使い方は`shared_ptr`化したいクラスで`std::enable_shared_from_this`をpublic継承し，`shared_from_this()`を呼ぶだけ．
しかし，**本機能を使っているクラスは必ずshared_ptrで管理されている必要がある**ので注意が必要．そうでない場合の動作は未定義．

```cpp
class A : public std::enable_shared_from_this {
A(){
  auto shared_ptr_this = shared_from_this();
}
};
```

### [std::future](https://ja.cppreference.com/w/cpp/thread/future)

非同期処理を行うためのクラス．このクラスでは今は無いが，「未来」に手に入るであろうデータを取り扱う．  
データが手に入ったかどうかは，`valid`関数で調べることができ，`get`関数で取り出せる．  
また，手に入るまで待機する`wait`関数も何種類か用意されている．

| 関数名       | 説明                |
|---------------|-------------------|
| `wait`       | 処理完了まで待つ             |
| `wait_for(<duration>)`        | 処理完了まで待つ（指定時間でタイムアウト）            |
| `wait_until(<time_point>)` | 処理完了まで待つ（指定時間になったらタイムアウト）          |

### [std::promise](https://cpprefjp.github.io/reference/future/promise.html)

`std::future`に出来上がったデータを受け渡す役割を持ったクラス  
使い方は以下のような感じ

```cpp
std::promise<int> p;
std::future<int> f = p.get_future();

p.set_value(1);

std::cout << f.get() << std::endl;
```

`std::primise`は，データを渡す相手に「今はまだ空だけど，将来ここにデータを送るからね」と言って`std::future`を渡す．  
後日，データを見つけたstd::promise`は`set_value`関数でデータを送り，`std::future`側では`get`関数で受けとれる．

### [委譲コンストラクタ(C++11)](https://cpprefjp.github.io/lang/cpp11/delegating_constructors.html)

C++11からの機能．複数のコンストラクタ内で共通の処理を初期化処理で行える．  
（コンストラクタの本体にメンバ関数を呼び出す方法もあるが，メンバ関数は初期化が完了した後にしか呼び出せない共通処理なので，ベタ書きしたり委譲コンストラクタを使う場合に比べてパフォーマンスで劣る）

```cpp
class X {
public:
  X(int i) { std::cout << i << std::endl; }
  X() : X(42) {}
};
```

### [初期化子リスト(C++11)](https://cpprefjp.github.io/lang/cpp11/initializer_lists.html)

以下のような，波括弧による初期化を可能とする'

```cpp
std::vector<int> v1 = {1, 2, 3};
std::vector<int> v2 {1, 2, 3};
```

初期化子リストを使うには，`std::initilizer_list`を引数とするコンストラクタが必要'

```cpp
class MyVector {
  std::vector<int> data_;
public:
  MyVector(std::initializer_list<int> init)
    : data_(init.begin(), init.end()) {}};
```

### [一様初期化(C++11)](https://cpprefjp.github.io/lang/cpp11/uniform_initialization.html)

`std::initilizer_list`以外のコンストラクタも波括弧を使った初期化ができる機能  
情報が足りていなかったりすると，`std::initializer_list`型として推論されてしまったりするので注意．

```cpp
struct X {
  X(int, double, std::string) {}
};
X createX(){return {1, 3.14, "hello"}; }
```

`std::initializer_list`のコンストラクタとそれ以外のコンストラクタの両方で受け取れるような波括弧リストを渡す時，前者が呼び出される．  
但し，空の初期化子リストを渡す場合はデフォルトコンストラクタが優先される．

### [インライン名前空間(C++11)](https://cpprefjp.github.io/lang/cpp11/inline_namespaces.html)

```cpp
inline namespace my_namespace{}
```

省略可能な名前空間を定義できる．  
上位の名前空間を`using`するだけで下位のinline名前空間の機能が使えるようになるので，本来なら複数の`using`文が必要な場合も，１つで事足りる．  

以下のように，複数バージョンの関数がそれぞれのバージョンの名前空間にある時，1つだけ`inline`名前空間で定義することでデフォルトのバージョンを表現することが出来る．  
また，デフォルトバージョンを変更する場合も`inline`名前空間を切り替えるだけで実現できる．  

```cpp
namespace api {
  inline namespace v1 {
    void f(){}
  }
  namespace v2 {
    void f(){}
  }
}
```

### [入れ子名前空間の定義(C++17)](https://cpprefjp.github.io/lang/cpp17/nested_namespace.html)

入れ子上になった名前空間を一度に定義することが可能になる

C++14まで

```cpp
namespace aa{
  namespace bb{
    void f(){}
  }
}
```

C++17から

```cpp
namespace aa::bb{
  void f(){}
}
```

### [入れ子名前空間でのインライン名前空間(C++20)](https://cpprefjp.github.io/lang/cpp20/nested_inline_mamespaces.html)

C++11からのインライン名前空間とC++17からの入れ子名前空間の併用は出来なかったが，C++20から出来るようになった

C++17

```cpp
namespace aa{
  inline namespace bb{
    void f(){}
  }
}
```

C++20

```cpp
namespace aa::inline bb{
  void f(){}
}
```

### [範囲for文(C++11)](https://cpprefjp.github.io/lang/cpp11/range_based_for.html)

for文を簡潔に書くことが出来る機能．

```cpp
for ( auto element : elements ) std::cout << element << std::endl;
```

↑は↓のように展開される（C++17以降や，配列を扱う時は展開され方が異なる）

```cpp
auto && __range = elements;
for (auto __begin = __range.begin(), __end = __range.end(); __begin != __end; ++__begin) {
  auto element = *__begin;
  std::cout << element << std::endl;
}
```

↑から分かるように，`begin()`,`end()`,`operator++()`, `operator*()`, `operator!=()`が適切に定義されていれば自作クラスでも使える．

注意点

- 要素をeraseしてはいけない(バグる)
- インデックスの取得とは相性が悪い（出来ないこともない：[別ページ](https://hansrobo.github.io/mycheatsheets/range-based-for#:~:text=%E7%AF%84%E5%9B%B2for%E6%96%87(range%2Dbased%20for)%E3%81%A7%E3%82%A4%E3%83%B3%E3%83%87%E3%83%83%E3%82%AF%E3%82%B9%E3%82%92%E4%BD%BF%E3%81%84%E3%81%9F%E3%81%84)参照）
- 範囲として渡した配列などが，範囲for文実行中に寿命が切れないように注意

### [範囲forループの制限緩和(C++17)](https://cpprefjp.github.io/lang/cpp17/generalizing_the_range-based_for_loop.html)

範囲for文を適用するコンテナの`begin()`関数と`end()`関数の戻り値の型が一致していなくても良くなった．  
正直使う機会は少ないが，番兵法アルゴリズムを使いたいときなどは便利に使えるらしい．  
使いたいから緩和したというより，制限する意味がなかったから緩和したという方が正しそう？  

### [範囲for文がカスタマイゼーションポイントを見つけるルールを緩和](https://cpprefjp.github.io/lang/cpp20/relaxing_the_range_for_loop_customization_point_finding_rules.html)

範囲for文を展開する時に`begin()`と`end()`を探すルールが変更された．（C++11まで遡って変更されていて，[gcc8,clang8以降で適用されている](https://cpprefjp.github.io/implementation-status.html#:~:text=%E3%81%97%E3%81%AB%E3%81%84%E3%81%8F-,8,8,-2019%20Update%205)）  

適用前イメージ

```cpp
if ( beginメンバ関数が存在 OR endメンバ関数が存在 ){
  beginメンバ関数を使用
}else{
  非メンバbegin/end関数を探す&使う
}
```

適用後イメージ

```cpp
if ( beginメンバ関数が存在 AND endメンバ関数が存在 ){
  begin/endメンバ関数を使用
}else{
  非メンバbegin/end関数を探す&使う
}
```

[cppref.jpのページ](https://cpprefjp.github.io/lang/cpp20/relaxing_the_range_for_loop_customization_point_finding_rules.html)のサンプルコードは説明が少なくてわかりにくいので，以下解説  

- そもそも`std_stringstream`には`begin()`と`end()`が存在しない
- しかし，`std_stringstream`が継承している`std::ios_base`には[`std::ios_base::end`](https://cpprefjp.github.io/reference/ios/ios_base/type-seekdir.html#:~:text=current%20%E3%81%AE%E7%95%A5%EF%BC%89-,end,-%E3%82%B9%E3%83%88%E3%83%AA%E3%83%BC%E3%83%A0%E3%81%AE%E7%8F%BE%E5%9C%A8)(関数ではない)が存在する
- `std::ios_base::end`が範囲for文に必要な`end`として認識される
- 適用前のルールだと，`begin`の存在を確認することなくメンバ関数で範囲forの展開をしてしまう
- 適用後のルールでは，`begin`の存在も確認を行うため，`std::ios_base::end`は範囲forの展開に使われない
- 適用後のルールでは結局非メンバの`begin`と`end`が使われて問題なくコンパイルできる

### [属性構文(C++11)](https://cpprefjp.github.io/lang/cpp11/attributes.html)

ソースコードに関する追加の情報をコンパイラに伝えるための構文  
主に，コンパイラの最適化のためと警告抑制のために使われる印象  
型などとは違い，ユーザが属性を定義することは出来ない．  

現在存在するのは以下

- [noreturn(C++11)](https://cpprefjp.github.io/lang/cpp11/attributes.html#noreturn)
- [carries_dependency(C++11)](https://cpprefjp.github.io/lang/cpp11/attributes.html#carries_dependency)
- [deprecated(C++14)](https://cpprefjp.github.io/lang/cpp14/deprecated_attr.html)
- [maybe_unused(C++17)](https://cpprefjp.github.io/lang/cpp17/maybe_unused.html)
- [nodicard(C++17)](https://cpprefjp.github.io/lang/cpp17/nodiscard.html)
- [fallthrough(c++17)](https://cpprefjp.github.io/lang/cpp17/fallthrough.html)
- [no_unique_adress(c++20)](https://cpprefjp.github.io/lang/cpp20/language_support_for_empty_objects.html)
- [likely, unlikely (c++20)](https://cpprefjp.github.io/lang/cpp20/likely_and_unlikely_attributes.html)

### [noreturn属性(C++11)](https://cpprefjp.github.io/lang/cpp11/attributes.html#noreturn)

⇒警告抑制タイプの属性：「関数が返らないパスが存在する」という警告を抑制  
⇒最適化促進タイプの属性：制約を増して最適化に寄与  

```cpp
[[noreturn]] void report_error(){
  throw std::runtime_error("error");
}
```

関数が決して返らない（=必ず例外が投げられる？）ことをコンパイラに伝える  
この属性がついた関数が返る場合の動作は未定義

### [carries_dependency属性(C++11)](https://cpprefjp.github.io/lang/cpp11/attributes.html#carries_dependency)

TBD  

TODO:

- [std::atomic](https://cpprefjp.github.io/reference/atomic/atomic.html)
- [std::memory_order](https://cpprefjp.github.io/reference/atomic/memory_order.html)

### [deprecated属性(C++14)](https://cpprefjp.github.io/lang/cpp14/deprecated_attr.html)

```cpp
[[deprecated("please use new_func() function")]] void old_func() {}
```

非推奨であることを示す属性．使うとコンパイル時に警告を発する  
警告で出すメッセージも指定できるので，代わりに使って欲しい機能を示すとより親切  
この属性が使える場所は以下の通り

- クラス
- 型の別名
- 変数
- 非静的メンバ変数
- 関数
- 列挙型
- テンプレートの特殊化

### [maybe_unused属性(C++17)](https://cpprefjp.github.io/lang/cpp17/maybe_unused.html)

⇒警告抑制タイプの属性：意図して使ってない要素に対してコンパイラに文句を言わせない

主に，各種宣言部分で色々使える

```cpp
class [[maybe_unused]] X;
using integer [[maybe_unused]] = int;
[[maybe_unused]] typedef int integer;
[[maybe_unused]] void f();
template <class T>
[[maybe_unused]] inline void f();
enum class [[maybe_unused]] E {
  A [[maybe_unused]],
  B
};
```
  
関数の引数の場合，maybe_unusedを指定せずとも，引数名を定義しないことで警告を抑制できる．  

```cpp
void func1([[maybe_unused]]int unused_arg){}
void func2(int){}
```

### [nodiscard属性(C++17)](https://cpprefjp.github.io/lang/cpp17/nodiscard.html)

「関数の戻り値を破棄してはいけない」という情報をコンパイラに伝え，エラー処理など本来無視してはならない戻り値の意図しない無視をプログラマに警告として伝えるための属性である．

関数に使用した場合，その返り値を無視すると警告を発するようになる．
クラス，構造体，列挙型の宣言に対して指定した場合，その型の返り値を無視すると警告を発する．

```cpp
struct [[nodiscard]] error_info{};
[[nodiscard]] int func(){return 0;}
```

C++20では更に以下の改良が加わっている

- [コンストラクタのオーバーロードごとに付加出来るように](https://cpprefjp.github.io/lang/cpp20/nodiscard_for_constructors.html)
- [警告の理由の文字列を付加できるように](https://cpprefjp.github.io/lang/cpp20/nodiscard_should_have_a_reason.html)

### [fallthrough属性(C++17)](https://cpprefjp.github.io/lang/cpp17/fallthrough.html)

switch-case文においてcaseの次にbreak文を置かないことで連続の複数のcaseに渡ってプログラムが実行されること（fallthrough）は，
しばしば悪用されるが，プログラマが見落としやすくバグにもつながりやすい．そのため，コンパイラはフォールスルーを検知すると警告を出す．
この`[[fallthrough]]`属性はフォールスルーが意図的であることを伝えると同時に，プログラマにとってもフォールスルーを見つけやすくするものである．

```cpp
switch (N) {
  case 1:
    // do something
    [[fallthrough]]
  case 2:
    break;
  default:
}
```

なお，最後のcaseやdefaultに記述するとコンパイルエラーとなる

### [optional::and_then (C++23)](https://cpprefjp.github.io/reference/optional/optional/and_then.html)

無効値チェックだけのためのifがなくせそう

```c++
std::optional<int> opt;

// before
if(opt){
	// do something
}

// after
opt.and_then([](int val){
	// do something
})
```

↑は1つだけだが，以下のような処理が複雑化したり連続したりすると威力を発揮しそう

```c++
// before
if(opt){
	if(opt.value != 2){
		// do something
	}
}
// after
opt.and_then([](int val){
	if(val ==2){
		return null_opt;
	}else{
		return val;
	}
}).and_then([](int val){
	// do something
})
```

### [指示付き初期化(C++20)](https://cpprefjp.github.io/lang/cpp20/designated_initialization.html)
```c++
struct Point3D { int x; int y; int z = 0; };
```
に対して
```c++
Point3D p2 {.x = 1, .y = 2, .z = 3};
```
と言ったメンバ名を指定しての初期化が可能になる

#### 用語
| 単語                   | 説明                                                                        |
| ---------------------- | --------------------------------------------------------------------------- |
| 指示子                 | `.x`とかのメンバ名を指定する書き方のこと                                    |
| 指示付き初期化子       | `.x = 1` や`.y{2}`などの指示子を使った初期化のこと                          |
| 指示付き初期化子リスト | `{.x = 1, .y = 2, .z = 3}` など指示付き初期化子が使われた初期化リストのこと |

#### 注意
- 指示付き初期化子の順番は定義した順番である必要がある
	- 違う場合はコンパイルエラー
- 初期化子は直接ネストしてアクセスできない
	- 構造体が入れ子になっている場合など
		- NG：{ .a.b = 1 }
		- OK：{ .a{ .b = 1 } }
#### 名前付き引数のようなもの
```c++
struct Fuga{
int a,b,c;
}
void hoge(Fuga fuga)
hoge({.a = 1, .b=2, .c=3})
```

### INVOKE要件

- 関数呼び出しを抽象化した，仮想操作”INVOKE”についての要件
- C++17からは `std::invoke`として実体化されている

#### 用語

| 用語              | 雑な要約                             | 例               |
| ----------------- | ------------------------------------ | ---------------- |
| call-signature    | 関数の宣言文の型情報だけ残したもの   | int (float, int) |
| callable-type     | 関数呼び出し演算子`()`を適用できる型 |                  |
| callable-object   | callable-type型のオブジェクト        |                  |
| call-wrapper-type |                                      |                  |
| call-wrapper      | call-wrapper-type型のオブジェクト    |                  |
| target-object                  |　callable-objectに保持されているオブジェクト| ラムダ式でキャプチャする変数などが該当する？                  |


### [std::invoke(C++17)](https://cpprefjp.github.io/reference/functional/invoke.html)

TBD

## [std::copysign(C++11)](https://cpprefjp.github.io/reference/cmath/copysign.html)

地味だけど結構うれしいやつ．
今まで，3項演算子やif文を使って符号判定してプラスマイナス反転させていた処理を簡潔に書くことができる．
```c++
auto value = std::copysign(<絶対値マン>, <符号マン>)
```


## [std::exchange(C++14)](https://cpprefjp.github.io/reference/utility/exchange.html)

値を書き換えて、書き換える前の値を返す。

```C++
auto <古い値の格納変数> = std::exchange(<書き換え対象変数>, <新しい値>)
```

後置インクリメント演算子みたいなイメージかも

```C++
int value = 0;
std::cout << value++ << std::endl; // output: 0
std::cout << std::exchange(value, value+1) std::endl; // output: 1
std::cout << value << std::endl; // output: 2
```


## [std::swap(C++11)](https://cpprefjp.github.io/reference/utility/swap.html)

2つの値を入れ替えるswap動作を行う。配列に対しても使える。

```C++
int a,b;
std::swap(a,b);
```

今まで以下のようにわざわざ一時変数を作って値を入れ替えていたのが1行でかける。

```C++
int a,b;
int tmp = a;
a = b;
b = tmp;
```

少し脇道にそれるが`std::exchange`を知っていたらこちらでもかける

```C++
int a,b;
a = std::exchange(b,a);
```

## [std::partition(C++03)](https://cpprefjp.github.io/reference/algorithm/partition.html)

条件を表す関数オブジェクトを指定して、条件を満たす（関数がtrueを返す）要素をコンテナの前の方へ、条件を満たさないものを後ろに集める。

```C++
std::vector<int> v = {1,2,3,4,5};
// 条件: 偶数
auto pos = std::partition(v.begin(), v.end(), [](int x){ return x % 2 == 0; });
// v: {4,2,3,1,5}, pos: 添字2に当たるイテレータ
```


## [std::mismatch(C++03)](https://cpprefjp.github.io/reference/algorithm/mismatch.html)

与えられた2つの範囲のうち、最初に一致しない位置を検索する。

```C++
std::vector<int> a = {1,2,3,4,5};
std::vector<int> b = {4,5,6,6,7};

// mod3が一致しない最初の位置
auto [a_pos, b_pos] = std::mismatch(a.begin(), a.end(), b.begin(), [](const int & a, const int & b){ return a%3 == b%3; });
// a_pos: 位置3, 内容4のイテレータ
// b_pos: 位置3, 内容6のイテレータ
```

- `==`演算子の代わりの関数オブジェクトを指定することもできる
- ミスマッチが見つからなかったときはこういう検索系関数のお決まり通り最後の要素を返す。
- 2つ目の範囲の最後は省略可（a.size() > b.size()のときは使っちゃダメ）

### Coming Soon

- [属性構文(C++11)](https://cpprefjp.github.io/lang/cpp11/attributes.html)
  - [fallthrough(c++17)](https://cpprefjp.github.io/lang/cpp17/fallthrough.html)
  - [no_unique_adress(c++20)](https://cpprefjp.github.io/lang/cpp20/language_support_for_empty_objects.html)
  - [likely, unlikely (c++20)](https://cpprefjp.github.io/lang/cpp20/likely_and_unlikely_attributes.html)
  - [属性の名前空間予約(c++20)](https://cpprefjp.github.io/lang/cpp20/reserving_attribute_namespaces_for_future_use.html)
- [不明な属性を無視する(C++17)](https://cpprefjp.github.io/lang/cpp17/non_standard_attributes.html)
- [スレッドローカルストレージ(C++11)](https://cpprefjp.github.io/lang/cpp11/thread_local_storage.html)
- [ブロックスコープを持つstatic変数初期化のスレッドセーフ化(C++11)](https://cpprefjp.github.io/lang/cpp11/static_initialization_thread_safely.html)
- [extern template(C++11)](https://cpprefjp.github.io/lang/cpp11/extern_template.html)
- [依存名に対するtypenameとtemplateの制限緩和(C++11)](https://cpprefjp.github.io/lang/cpp11/dependent_name_specifier_outside_of_templates.html)
- [可変引数テンプレート](https://cpprefjp.github.io/lang/cpp11/variadic_templates.html)
- [継承コンストラクタ(C++11)](https://cpprefjp.github.io/lang/cpp11/inheriting_constructors.html)
- mutex
- lock_gard
