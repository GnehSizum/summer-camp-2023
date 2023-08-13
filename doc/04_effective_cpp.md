# Effective C++ (1)

## 1.让自己习惯C++

### 01-视C++为一个语言联邦

C++已经是个多重范型编程语言，一个同时支持过程形式（procedural）、面向对象形式（object-oriented）、函数形式（functional）、泛型形式（generic）、元编程形式（metaprogramming）的语言。

- **C** ：C++仍是以C为基础的。区块(blocks)、语句(statements)、预处理器(preprocessor)、内置数据类型(built-in data types)、数组(arrays)、指针(pointers)等统统来自C。许多时候C++对问题的解法其实不过就是较高级的C解法。
- **Object-Oriented C++** ：这部分也就是C with Classes所诉求的：classes(包括构造函数和析构函数)，封装(encapsulation)、继承(inheritance)、多态(polymorphism)、virtual函数（动态绑定）…等等。这一部分是面向对象设计之古典守则在C++上的最直接实施。
- **Template C++** ：这是C++的泛型编程部分，也是大多数程序员经验最少的部分。templates威力强大，它们带来崭新的编程范型，也就是所谓的 template metaprogramming(TMP,模板元编程)。但除非你是template激进团队的中坚骨干，大可不必太担心这些。TMP相关规则很少与C++主流编程互相影响。
- **STL** ：STL是个template程序库，看名称也知道，但它是非常特殊的一个。它对容器(containers)、迭代器(iterators).、算法(algorithms)以及函数对象(function objects)的规约有极佳的紧密配合与协调。STL有自己特殊的办事方式，当你使用STL工作，你必须遵守它的规约。

因此当这四个次语言相互切换的时候，可以更多地考虑高效编程，例如pass-by-value和pass-by-reference在不同语言中效率不同。

可见C++并不是一个带有一组守则的一体语言，它是四个次语言组成的联邦政府，每个次语言都有自己的规约。记住这四个次语言你就会发现C++容易了解得多。

### 02-尽量以const替换#define

应该让编译器代替预处理器定义，因为预处理器定义的变量并没有进入到symbol table里面。编译器有时候会看不到预处理器定义。

所以用

```cpp
const double Ratio = 1.653;
```

来替代

```cpp
#define Ratio 1.653
```

当我们以常量替换#defines，有两种特殊情况值得说说。

第一是定义常量指针。由于常量定义式通常被放在头文件内（以便被不同的源码含入），因此有必要将指针（而不只是指针所指之物）声明为const。例如若要在头文件内定义一个常量的（不变的）char*-based字符串，你必须写const两次：

```cpp
const char* const authorName = "name";
```

第二个值得注意的是class专属常量。为了将常量的作用域限制于class内，你必须让它成为class的一个成员；而为确保此常量至多只有一份实体，你必须让它成为一个static成员：

```cpp
class Gameplayer {
private:
    static const int NumTurns = 5;  //常量声明式
    int scores[NumTurns];           //使用该常量
};
```

然而你所看到的是NumTurns的声明式而非定义式。通常C++要求你对你所使用的任何东西提供一个定义式，但如果它是个class专属常量又是static且为整数类型(integral type,例如ints,chars,,bools)，则需特殊处理。只要不取它们的地址，你可以声明并使用它们而无须提供定义式。但如果你取某个class专属常量的地址，或纵使你不取其地址而你的编译器却（不正确地）坚持要看到一个定义式，你就必须另外提供定义式如下：

```cpp
const int GamePlayer::NumTurns;  //NumTurns的定义
                                 //下面告诉你为什么没有给予数值
```

请把这个式子放进一个实现文件而非头文件。由于class常量已在声明时获得初值（例如先前声明NumTurns时为它设初值5），因此定义时不可以再设初值。

旧式编译器也许不支持上述语法，它们不允许static成员在其声明式上获得初值。此外所谓的“in-class初值设定”也只允许对整数常量进行。如果你的编译器不支持上述语法，你可以将初值放在定义式：

```cpp
class CostEstimate {
private:
    static const double FudgeFactor;  //static class常量声明
                                      //位于头文件内
};

const double CostEstimate::FudgeFactor=1.35;  //static class常量定义
                                              //位于实现文件内
```

### 03-尽可能使用const

const的奇妙是，它允许你指定一个语义约束（也就是指定一个“不该被改动”的对象），而编译器会强制实施这项约束。它允许你告诉编译器和其他程序员某值应该保持不变。只要这（某值保持不变）是事实，你就该确实说出来，因为说出来可以获得编译器的襄助，确保这条约束不被违反。

关键字const多才多艺。你可以用它在class外部修饰global或namespace作用域中的常量，或修饰文件、函数、或区块作用域中被声明为static的对象。你也可以用它修饰class内部的static和non-static成员变量。面对指针，你也可以指出指针自身、指针所指物，或两者都（或都不）是const。

const语法虽然变化多端，但并不莫测高深。如果关键字const出现在星号左边，表示被指物是常量：如果出现在星号右边，表示指针自身是常量；如果出现在星号两边，表示被指物和指针两者都是常量。

如果被指物是常量，有些程序员会将关键字const写在类型之前，有些人会把它写在类型之后、星号之前。两种写法的意义相同，所以下列两个函数接受的参数类型是一样的：

```cpp
void fl(const Widget* pw);   //f1获得一个指针，指向一个常量的（不变的）Widget对象，
void f2(Widget const * pw);  //f2也是
```

两种形式都有人用，你应该试着习惯它们。

const最具威力的用法是面对函数声明时的应用。在一个函数声明式内，const可以和函数返回值、各参数、函数自身（如果是成员函数）产生关联。令函数返回一个常量值，往往可以降低因客户错误而造成的意外，而又不至于放弃安全性和高效性。举个例子，考虑有理数的operator*声明式：

```cpp
class Rational{···};
const Rational operator*(const Rational&lhs,const Rational&rhs);
```

至于const参数，没有什么特别新颖的观念，它们不过就像local const对象一样，你应该在必要使用它们的时候使用它们。除非你有需要改动参数或local对象，否则请将它们声明为const。只不过多打6个字符，却可以省下恼人的错误。

### 04-确定对象被使用前已先被初始化

如果你这么写：
```cpp
int x;
```

在某些语境下x被初始化（为0），但在其他语境中却不保证。如果你这么写：

```cpp
class Point {
    int  x, y;
};

Point p;
```

p的成员变量有时候被初始化（为0），有时候不会。请小心，因为这颇为重要。

读取未初始化的值会导致不明确的行为。在某些平台上，仅仅只是读取未初始化的值，就可能让你的程序终止运行。更可能的情况是读入一些“半随机”bits，污染了正在进行读取动作的那个对象，最终导致不可测知的程序行为，以及许多令人不愉快的调试过程。

初始化责任落在构造函数身上。规则很简单：确保每一个构造函数都将对象的每一个成员初始化。

这个规则很容易奉行，重要的是别混淆了赋值和初始化。考虑一个用来表现通讯簿的class，如下：


