# string 前补零

> #include <iostream>
> #include <sstream>
> #include <iomanip>
> using namespace std;
> void main()
> {
> int num = 1024;
> stringstream ss;
> ss << setw(5) << setfill('0') << num ;
> string str;
> ss >> str;         //将字符流传给 str
> //str = ss.str();  //也可以
> cout << str;
> }

## 得到当前时间

```
#include <iostream>
#include <ctime>

int main ()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo);
  std::string str(buffer);

  std::cout << str;

  return 0;
}
```

## 写入文件append

```
#include <fstream>

int main() {  
  std::ofstream outfile;

  outfile.open("test.txt", std::ios_base::app); // append instead of overwrite
  outfile << "Data"; 
  return 0;
}
```

## float to string

`to_string(float)`

## string to float

`stof(str)`

## 读取文档每一行内容

```
fstream newfile;
newfile.open(learning_file,ios::in);
if (newfile.is_open())
{
    string tp;
    while(getline(newfile, tp))
    {
        cout << tp << "\n";
    }
    newfile.close();
}
```

## 文件清空

```
fstream newfile;
newfile.open(learning_file,ios::out);
newfile.close();
```

## 字符串ｓｐｌｉｔ

```
#include <boost/algorithm/string.hpp>

std::string text = "Let me split this into words";
std::vector<std::string> results;

boost::split(results, text, [](char c){return c == ' ';});
```

## CMakeLists.txt 引入ｂｏｏｓｔ

```
find_package(Boost COMPONENTS program_options filesystem REQUIRED )
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries(learning ${catkin_LIBRARIES} ${Boost_LIBRARIES})
```

## explicit

```
#include <iostream> 

using namespace std; 

class Complex 
{ 
private: 
	double real; 
	double imag; 

public: 
	// Default constructor 
	Complex(double r = 0.0, double i = 0.0) : real(r), imag(i) {} 

	// A method to compare two Complex numbers 
	bool operator == (Complex rhs) { 
	return (real == rhs.real && imag == rhs.imag)? true : false; 
	} 
}; 

int main() 
{ 
	// a Complex object 
	Complex com1(3.0, 0.0); 

	if (com1 == 3.0) 
	cout << "Same"; 
	else
	cout << "Not Same"; 
	return 0; 
} 
```

程序输出　`Same`
在C ++中，如果类具有可以用单个参数调用的构造函数，则此构造函数将成为转换构造函数，因为这样的构造函数允许将单个参数转换为正在构造的类。
我们可以避免这种隐式转换，因为它们可能导致意外的结果。我们可以在explicit关键字的帮助下使构造函数显式化。\

```
using namespace std; 
  
class Complex 
{ 
private: 
    double real; 
    double imag; 
  
public: 
    // Default constructor 
    explicit Complex(double r = 0.0, double i = 0.0) : real(r), imag(i) {} 
  
    // A method to compare two Complex numbers 
    bool operator== (Complex rhs) { 
       return (real == rhs.real && imag == rhs.imag)? true : false; 
    } 
}; 
  
int main() 
{ 
    // a Complex object 
    Complex com1(3.0, 0.0); 
  
    if (com1 == (Complex)3.0) 
       cout << "Same"; 
    else
       cout << "Not Same"; 
     return 0; 
} 
```

## Use Eigen in cmake program

```
```bash
find_package(Eigen3 3.3 REQUIRED)
...
include_directories(${EIGEN3_INCLUDE_DIR})
...
target_link_libraries(example Eigen3::Eigen)
```

## double to string 保留小数点后1位

```cpp
std::string doubleToString(const double &val)

{

    char* chCode;

    chCode = new char[20];

    sprintf(chCode, "%.2lf", val);

    std::string str(chCode);

    delete[]chCode;

    return str;

}
```
