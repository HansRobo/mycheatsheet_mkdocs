---
created: 2024-02-29 10:41:27
---

## パラメータ

### Nodeからいじる

#### get_parameter

##### 1

```
rclcpp::Parameter get_parameter(const std::string & name) const
```

名前を指定して、パラメータを返す。
存在しないパラメータに対しては `rclcpp::exceptions::ParameterNotDeclaredException`を送出する。

##### 2
```
bool get_parameter(const std::string & name, rclcpp::Parameter & parameter) const
```

名前を指定して、引数のパラメータ参照に代入する。
存在しないパラメータの場合、falseが返る。

##### 3

```
template<typename ParameterT>
bool get_parameter(const std::string & name, ParameterT & parameter) const
```

2のパラメータ型をテンプレートで指定するバージョン。
名前を指定して、引数の生パラメータ型の参照に代入する。
存在しないパラメータの場合、falseが返る。
テンプレートで指定したパラメータ型と取得しようとしているパラメータの型が異なる場合、`rclcpp::ParameterTypeException`を送出する。

##### 4
```
template<typename ParameterT>
bool
get_parameter_or(
  const std::string & name,
  ParameterT & parameter,
  const ParameterT & alternative_value) const
```

3のラッパーで、3の返り値がfalseのときに3番目の引数を返す。
パラメータが宣言されていないときに自動で宣言してくれるような機能はない。
返り値はROS空間からパラメータを取得できたかどうか（=alternative_valueが使われたときはfalseが返る）
他のget_parameterと違って実装はnode_impl.hppにあるので注意。

##### 5

```
template<typename ParameterT>
ParameterT
get_parameter_or(
  const std::string & name,
  const ParameterT & alternative_value) const
```

4のラッパーになっていて、パラメータを参照ではなく返り値で返してくれる。