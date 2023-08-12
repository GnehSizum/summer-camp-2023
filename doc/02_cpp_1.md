# C++多文件编程

## 1.头文件

C++源文件.cpp并不是C++程序中唯一常见的文件。另一种类型的文件被称为头文件。头文件的扩展名通常是.h，但你偶尔也会看到.hpp，或者根本就没有扩展名。

头文件的主要目的是将声明传播到代码文件，允许我们把声明放在一个地方而在我们需要的时候导入它们。

最常见的例子，`#include <iostream>` 中的iostream就是头文件，包含它在内的一系列头文件从属于标准库，`std::cout` 就是在标准库中声明的。

当涉及到函数和变量时，值得注意的是，**头文件通常只包含函数和变量的声明**，而不包含函数和变量的定义（否则可能导致违反单一定义规则，见下）。

### 1-1.为什么头文件不包含定义

来看这个例子：

```cpp
// dog.h
int dog = 233;

//dog.cpp
#include "dog.h"

// main.cpp
#include "dog.h"
#include <iostream>
int main()
{
    std::cout << dog;
    return 0;
}
```

此时会引发链接错误。原因在于，dog.cpp与main.cpp都包含了dog.h，于是两个源文件都含有dog的定义，导致重定义错误。

在多文件编程中，多个源文件包含一个头文件的情况是很常见的，如果在头文件中进行定义，难免会发生上面的情况。

### 1-2.自定义头文件示例

一般而言，头文件会与源文件相对应（显然，因为头文件里是声明，源文件里是对应的定义）。于是我们创建add.h与add.cpp如下：

```cpp
//add.h
int add(int x, int y);

//add.cpp
#include "add.h"
int add(int x, int y)
{
    return x + y;
}
```

现在，我们就可以在main.cpp中使用add函数：
```cpp
//main.cpp
#include "add.h"
#include <iostream>
int main()
{
    std::cout << "The sum of 3 and 4 is " << add(3, 4) << endl;
    return 0;
}
```

## 2.多文件编程书写规范

### 2-1.不要再写using namespace std了

using的意义是，在整个文件中对应的函数名都默认在其后的命名空间中，using namespace std就是在整个文件范围内指定使用标准命名空间（如果不存在歧义）。

当然，这能帮我们少打很多琐碎的 `std::` ，如果你可以确保不会使用来自其他命名空间的同名函数，就可以大胆使用using。但除此之外的情况，为了尽可能降低bug出现的概率， **不要使用using namespace std！**

来看这个例子：

```cpp
#include <iostream>
using namespace std;

int cout()
{
    return 233;
}

int main()
{
    cout << "Hello, world!"; // Compile error! Which cout do we want here? The one in the std namespace or the one we defined above?
    return 0;
}
```

以上代码中，我们本打算使用自己定义的cout函数（我知道这看起来很蠢，但别忘了std中还有min、max等经常可能被用来自己定义函数的函数名字），但由于我们using namespace std，编译器不能分辨我们究竟要使用哪个命名空间的cout，于是报错。在多文件编程中，这种错误的出现会让人摸不着头脑，难以定位错误所在。

### 2-2.源文件应导入对应的头文件

这样做允许编译器在编译期就能发现错误，而非到链接期才能发现。

```cpp
// a.h
int something(int); // return type of forward declaration is int

// a.cpp
#include "a.h"
void something(int) // error: wrong return type
{
    //code
}
```

由于源文件导入了头文件，返回值类型不匹配的错误在编译期即可发现（很多IDE的语法检查也可以发现），方便我们debug。

### 2-3.使用双引号来导入自定义的头文件

使用尖括号时，它告诉预处理器这不是我们自定义的头文件。编译器将只在包含目录（include directories）所指定的目录中搜索头文件。包含目录是作为项目/IDE设置/编译器设置的一部分来配置的，通常默认为编译器和（或）操作系统所带头文件的目录。编译器不会在你项目的源代码目录中搜索头文件。

使用双引号时，我们告诉预处理器这是我们自定义的头文件。编译器将首先在当前目录下搜索头文件。如果在那里找不到匹配的头文件，它就会在包含目录中搜索。

因此，**使用双引号来包含你编写的头文件，或者预期在当前目录中可以找到的头文件**。使用尖括号来包含编译器、操作系统或安装的第三方库所附带的头文件。

### 2-4.显式包含所需的所有头文件

**每个文件都应该明确地#include它所需要的所有头文件来进行编译。不要依赖从其他头文件中转来的头文件。**

如果你包含了a.h，而a.h中包含了b.h，则b.h为隐式包含，a.h为显式包含。你不应该依赖通过这种方式包含的b.h的内容，因为头文件的实现可能会随着时间的推移而改变，或者在不同的系统中是不同的。而这一旦发生，你的代码就可能不能在别的系统上编译，或者将来不能编译。直接同时显式包含a.h和b.h就能解决这个问题。

### 2-5.给自定义头文件加上header guard

前面提到，如果在一个文件中多次定义一个变量/函数会引发重定义报错。现在来看这个例子：

```cpp
// aaa.h
int add()
{
    return 233;
}

// bbb.h
#include "aaa.h"

// main.cpp
#include "aaa.h"
#include "bbb.h"
int main()
{
    return 0;
}
```

在main.cpp中，add()通过两次包含被定义了两次，将引发重定义问题。关键的问题在于，只站在main.cpp的角度来看，它很无辜，只不过是普通地包含了两个不同的头文件而已。因此，解决这个问题应该从头文件本身下手。这就是我们需要header guard的原因。

header guard的格式如下：

```cpp
#ifndef ROBORTS_DECISION_PATROL_H
#define ROBORTS_DECISION_PATROL_H
// your code here
#endif
```

这样，就可以保证相同的部分只会被包含一次。

现代编译器往往提供另一种更简洁的header guard—— `#pragma once` 。

```cpp
#pragma once

// your code here
```

它的效果与传统的header guard相同，而且更加简单。但我仍然建议使用传统的header guard，因为 `#pragma once` 可能在某些编译器上并不受支持。

### 2-6.严禁循环包含

来看这样一个例子：

```cpp
// b.h
#include "a.h"
// ...

// a.h
#include "b.h"
// ...
```

未使用header guard时，这样的写法会引发连锁反应：b.h包含a.h，后者包含b.h，再包含a.h......

即使使用了header guard消除了这种连锁反应，也会出现错误——如果b包含a，那么a包含b的代码则会因为header guard在预处理阶段就失去作用，于是a没能成功包含b，则a无法看到其需要的b中的声明。

因此， **禁止循环包含！** （此外，出现循环包含时有很大的可能是因为你的代码设计不好，例如出现了循环依赖）

<br />

## Reference

[Learn C++ – Skill up with our free tutorials (learncpp.com)](https://www.learncpp.com/)