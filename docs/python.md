## conda The following packages are not available from current channels:
`conda config --append channels conda-forge`\
It tells conda to also look on the conda-forge channel when you search for packages. 

## conda 创建环境
`conda create -n xception_net python==3.6.5 numpy==1.17.4 scipy==1.3.3 h5py==2.10.0 Keras==2.3.1 tensorflow-gpu==1.15.0`

As the comment at the top indicates, the output of\
`conda list -e > requirements.txt`

can be used to create a conda virtual environment with\
`conda create --name <env> --file requirements.txt`

## cudnn_status_internal_error tensorflow
You can try Allowing GPU memory growth with:
```
import tensorflow as tf
gpu = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(gpu[0], True)
```

## vscode activate conda env
```
{
    "ros.distro": "melodic",
    "python.autoComplete.extraPaths": [
        "/home/pmjd/Disk/anaconda3/envs/wjj/lib/python3.6/site-packages"
    ],
    "python.terminal.activateEnvInCurrentTerminal": true,
    "python.condaPath": "/home/pmjd/Disk/anaconda3/bin/conda",
    "python.defaultInterpreterPath": "/home/pmjd/Disk/anaconda3/envs/wjj/bin/python"
}
```
## NameError: name 'xrange' is not defined
```
try:
    # Python 2
    xrange
except NameError:
    # Python 3, xrange is now named range
    xrange = range
```
## TensorFlow ValueError: Cannot feed value of shape (64, 64, 3) for Tensor u'Placeholder:0', which has shape '(?, 64, 64, 3)'
image has a shape of (64,64,3).\
Your input placeholder _x have a shape of (?, 64,64,3).\
The problem is that you're feeding the placeholder with a value of a different shape.\
You have to feed it with a value of (1, 64, 64, 3) = a batch of 1 image.\
Just reshape your image value to a batch with size one.
`np.expand_dims(img, axis=0)`

## opencv conda
opencv is not compatible with python 3. I had to install opencv3 for python 3. The marked answer in how could we install opencv on anaconda? explains how to install opencv(3) for anaconda:

Run the following command:

`conda install -c https://conda.binstar.org/menpo opencv`

I realized that opencv3 is also available now, run the following command:

`conda install -c https://conda.binstar.org/menpo opencv3`

Edit on Aug 18, 2016: You may like to add the "menpo" channel permanently by:

`conda config --add channels menpo`

And then opencv can be installed by:

`conda install opencv (or opencv3)`

Edit on Aug 14, 2017: "clinicalgraphics" channel provides relatively newer vtk version for very recent python3

`conda install -c clinicalgraphics vtk`

Edit on April 16, 2020 (based on @AMC's comment): OpenCV can be installed through conda-forge (details see here)

`conda install -c conda-forge opencv`

## pointcloud2 to array 
```
def pointcloud2_to_array(cloud_msg, squeeze=True):
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)
    cloud_arr = np.fromstring(cloud_msg.data, dtype_list)

    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width)) 
```

## read pcd to array 
```
import numpy as np 
import open3d as o3d

pcd = o3d.io.read_point_cloud("pointcloud_path.pcd")
out_arr = np.asarray(pcd.points)  
print ("output array from input list : ", out_arr)  
```

## kitti 数据集下载
1. 图片下载
> https://s3.eu-central-1.amazonaws.com/avg-kitti/data_object_image_2.zip
2. 点云下载
> https://s3.eu-central-1.amazonaws.com/avg-kitti/data_object_velodyne.zip
3. 标签下载
> https://s3.eu-central-1.amazonaws.com/avg-kitti/data_object_label_2.zip
4. 矫正文件下载
> https://s3.eu-central-1.amazonaws.com/avg-kitti/data_object_calib.zip

## python "//" operator
In python 2.x
```
>>> 10/3
3
>>> # To get a floating point number from integer division:
>>> 10.0/3
3.3333333333333335
>>> float(10)/3
3.3333333333333335
```
In python 3.x
```
>>> 10/3
3.3333333333333335
>>> 10//3
3
```
## python super()
内置的super()返回一个代理对象（超类的临时对象),该代理对象允许我们访问基类的方法。\
在python中，super()有两个主要用例：
* 让我们避免显示使用基类名称
* 处理多重继承\
#### 示例１：具有单继承的super()
在单继承的情况下，它允许我们通过引用基类super()
```
class Mammal(object):
  def __init__(self, mammalName):
    print(mammalName, 'is a warm-blooded animal.')
    
class Dog(Mammal):
  def __init__(self):
    print('Dog has four legs.')
    super().__init__('Dog')
    
d1 = Dog()
```
输出
```
Dog has four legs.
Dog is a warm-blooded animal.
```
该super()内建返回一个代理对象，替代对象，可以通过委托调用基类的方法，这称为“间接"(使用引用基础对象的能力super())\
由于间接是在运行时计算的，因此我们可以在不同的时间使用不同的基类（如果需要）
#### 示例２：具有多重继承的super()
```
class Animal:
  def __init__(self, Animal):
    print(Animal, 'is an animal.');

class Mammal(Animal):
  def __init__(self, mammalName):
    print(mammalName, 'is a warm-blooded animal.')
    super().__init__(mammalName)
    
class NonWingedMammal(Mammal):
  def __init__(self, NonWingedMammal):
    print(NonWingedMammal, "can't fly.")
    super().__init__(NonWingedMammal)

class NonMarineMammal(Mammal):
  def __init__(self, NonMarineMammal):
    print(NonMarineMammal, "can't swim.")
    super().__init__(NonMarineMammal)

class Dog(NonMarineMammal, NonWingedMammal):
  def __init__(self):
    print('Dog has 4 legs.');
    super().__init__('Dog')
    
d = Dog()
print('')
bat = NonMarineMammal('Bat')
```
输出
```
Dog has 4 legs.
Dog can't swim.
Dog can't flay.
Dog is a warm-blooded animal.
Dog is an animal.

Bat can't swim.
Bat is a warm-blooded animal.
Bat is an animal.
```
#### Method Resolution Order方法解析顺序 (MRO)
```
>>> Dog.__mro__
(<class 'Dog'>, 
<class 'NonMarineMammal'>, 
<class 'NonWingedMammal'>, 
<class 'Mammal'>, 
<class 'Animal'>, 
<class 'object'>)
```

## Split method in python is outputing an index error
`one of your lines must be empty`

## deque in python ｐｙｔｈｏｎ中的双端队列
```
# Python code to demonstrate deque 
from collections import deque 
# Declaring deque 
queue = deque(['name','age','DOB']) 
print(queue) 
=========================================================
Output:
deque(['name', 'age', 'DOB'])
```
- append() :- This function is used to insert the value in its argument to the right end of deque.
- appendleft() :- This function is used to insert the value in its argument to the left end of deque.
- pop() :- This function is used to delete an argument from the right end of deque.
- popleft() :- This function is used to delete an argument from the left end of deque.
- index(ele, beg, end) :- This function returns the first index of the value mentioned in arguments, starting searching from beg till end index.
- insert(i, a) :- This function inserts the value mentioned in arguments(a) at index(i) specified in arguments.
- remove() :- This function removes the first occurrence of value mentioned in arguments.
- extend(iterable) :- This function is used to add multiple values at the right end of deque. The argument passed is an iterable.
- extendleft(iterable) :- This function is used to add multiple values at the left end of deque. The argument passed is an iterable. Order is reversed as a result of left appends.
- reverse() :- This function is used to reverse order of deque elements.
- rotate() :- This function rotates the deque by the number specified in arguments. If the number specified is negative, rotation occurs to left. Else rotation is to right.

### numpy.argmax
numpy.argmax(a, axis=None, out=None)[source]\
Returns the indices of the maximum values along an axis.\
**parameters**\
- **a: array_like**,Input array.
- **axis:int, optional**, By default, the index is into the flattened array, otherwise along the specified axis.
- **outarray, optional**, If provided, the result will be inserted into this array. It should be of the appropriate shape and dtype.

**Returns**\
**index_array:ndarray of ints**\
Array of indices into the array. It has the same shape as a.shape with the dimension along axis removed.

![1](images/Screenshot%202021-01-22%2013:52:21.png)\
![1](images/Screenshot%202021-01-22%2013:53:11.png)

### python random.sample()
![1](images/Screenshot%202021-01-22%2013:57:52.png)

### numpy.amax()
![1](images/Screenshot%202021-01-22%2014:13:38.png)

### python pip 不能用
`PIP_NO_CACHE_DIR=off pip install gym`

## num.linspace() in Python
```
import numpy as np
print("B\n", np.linspace(2.0, 3.0, num=5, retstep=True),"\n")

x = np.linspace(0, 2, 10)
print("A\n", np.sin(x))
```
Output
```
B
 (array([ 2.  ,  2.25,  2.5 ,  2.75,  3.  ]), 0.25)

A
 [ 0.          0.22039774  0.42995636  0.6183698   0.77637192  0.8961922
  0.9719379   0.99988386  0.9786557   0.90929743]
```