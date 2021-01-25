### tf.compat.v1.train.Saver
> **tf.compat.v1.train.Saver**\
> Saves and restores还原 variables
```
tf.compat.v1.train.Saver(
    var_list=None, 
    reshape=False, 
    sharded=False, 
    max_to_keep=5,
    keep_checkpoint_every_n_hours=10000.0, 
    name=None, 
    restore_sequentially=False,
    saver_def=None, 
    builder=None, 
    defer_build=False, 
    allow_empty=False,
    write_version=tf.train.SaverDef.V2, pad_step_number=False,
    save_relative_paths=False, 
    filename=None
)
```
> The **Saver** class adds ops行动 to save and restore variables to and form checkpoints.It also provides convenience method to run these ops.

> Checkpoints are binary files in a proprietary所有权 format which map variable names to tensor values.Checkpoints 是专有格式的二进制文件，该文件将变量名映射到张量值.The best way to examine审查 the contents of a checkpoints is to load it using a **Saver**

> **var_list** A list of Variable/SaveableObject, or a dictionary mapping names to SaveableObjects.If None,defaults to the list of all saveable objects.

> **reshape** If True,allows restoring恢复 parameters from a checkpoint where the variables have a different shape.

> **shareded** If True,shard the checkpoints,one per device每个设备一个

> **max_to_keep** Maximum number of recent checkpoints to keep,Default to 5

> **keep_checkpoint_every_n_hours** How often to keep checkpoints,Defaults to 10,000 hours.

> **name** String, Optional name to use as a prefix when adding operations.

> **restore_sequentially** A Bool,which if true,causes restore of different variables to happen sequentially within each device.This can lower memory usage when restoring large models.

> **saver_def** Optional SaverDef proto原型 to use instead of running the builder.This is only useful for specialty code that wants to recreate a Saver object for a previously built Graph that had a Saver.The saver_def proto should be the one returned by the as_saver_def() call of the Saver that was created for that Graph.

> **builder** Optional SaverBuilder to use if a saver_def was not provided. Default to BuikSaverBuilder().

> **defer_build** If True,defer adding the save and restore ops to the build() call. In that case build() should be called before finalizing定案 the graph or using the saver.

> **allow_empty** If False(defalut) rasie an error if there are no variables in the graph.Otherwise, construct the saver anyway and make it a no-op.

> **write_version** controls what format to use when saving checkpoints. It also affects certain filepath matching logic. The V2 format is the recommended choice: it is much more optimized than V1 in terms of memory required and latency incurred during restore. Regardless of this flag, the Saver is able to restore from both V2 and V1 checkpoints.

> **pad_step_number** if True, pads the global step number in the checkpoint filepaths to some fixed width (8 by default). This is turned off by default.

> **save_relative_paths** If True, will write relative paths to the checkpoint state file. This is needed if the user wants to copy the checkpoint directory and reload from the copied directory.

> **filename** 	If known at graph construction time, filename used for variable loading/saving.

### tf.compat.v1.variable_scope
> **tf.compat.v1.variable_scope**\
> A **context manager** for define ops that create variables(layers)用于定义创建变量（层）的操作的上下文管理器

```
tf.compat.v1.variable_scope(
    name_or_scope, 
    default_name=None, 
    values=None, 
    initializer=None,
    regularizer=None, 
    caching_device=None, 
    partitioner=None, 
    custom_getter=None,
    reuse=None, 
    dtype=None, 
    use_resource=None, 
    constraint=None,
    auxiliary_name_scope=True
)
```
> This context manager validates验证 that the values are from the same graph,ensures that graph is the default graph,and pushes a name scope范围 and a variable scope.

> Variable scope allows you to create new variables and to share already created ones while providing checks to not create or share by accident.变量作用域允许您创建新变量并共享已创建的变量，同时提供检查以防止意外创建或共享.

> Keep in mind that the counters for default_name are discarded丢弃 once the parent scope is exited. Therefore when the code re-enters the scope (for instance by saving it), all nested嵌套的 default_name counters will be restarted.

> Note that **reuse** flag is inherited: if we open a resuing scope,then all its sub-scope become reusing as well.

A note about name scoping:Setting **reuse** does not impact the naming of other ops such as mult.

> **reuse** True,None,or tf.compat.v1.AUTO_REUSE;if True, we go into reuse mode for this scope as well as all sub-scopes; if tf.compat.v1.AUTO_REUSE, we create variables if they do not exist, and return them otherwise; if None, we inherit the parent scope's reuse flag. When eager execution is enabled, new variables are always created unless an EagerVariableStore or template is currently active.

### tf.compat.v1.layers.dense
> **tf.compat.v1.layers.dense**\
> Functional interface for the densely-connected layer.

```
tf.compat.v1.layers.dense(
    inputs, units, activation=None, use_bias=True, kernel_initializer=None,
    bias_initializer=tf.zeros_initializer(), kernel_regularizer=None,
    bias_regularizer=None, activity_regularizer=None, kernel_constraint=None,
    bias_constraint=None, trainable=True, name=None, reuse=None
)
```

> The layer implement the operation:`output=activation(inputs * kernel + bias)` where activation is the activation function passed as the activation argument(if not **Ｎone**),kernel is a weights matrix created by the layer, and bias is a bias vector created by the layer(only if use_bias is True).

> **inputs** Tensor input.

> **units** Integer or Long, dimensionality of the output space.

> **activation** Activation function(callable), Set it to None to maintain a linear activation.

> **use_bias** Boolean, whether the layer uses a bias

> **kernel_initializer** Initializer function for the weight matrix. If None (default), weights are initialized using the default initializer used by tf.compat.v1.get_variable.

> **bias_initializer** Initializer function for the bias

> **kernel_regularizer** Regularizer正则化器 function for the weight matrix

> **bias_regularizer** Regularizer function for the bias.

> **activity_regularizer** Regularizer function for the output.

> **kernel_constraint** An optional projection function to be applied to the kernel after being updated by an Optimizer (e.g. used to implement norm constraints or value constraints for layer weights). The function must take as input the unprojected variable and must return the projected variable (which must have the same shape). Constraints约束条件 are not safe to use when doing asynchronous distributed training.

> **bias_constraint** An optional projection function to be applied to the bias after being updated by an Optimizer.

> **trainable** Boolean, if True also add variables to the graph collection GraphKeys.TRAINABLE_VARIABLES (see tf.Variable).

> **name** 	String, the name of the layer.

> **reuse** Boolean, whether to reuse the weights of a previous layer by the same name.

### tf.compat.v1.layers.BatchNormalization
> **tf.compat.v1.layers.BatchNormalization**\
> Batch Normalization layer

```
tf.compat.v1.layers.BatchNormalization(
    axis=-1, momentum=0.99, epsilon=0.001, center=True, scale=True,
    beta_initializer=tf.zeros_initializer(),
    gamma_initializer=tf.ones_initializer(),
    moving_mean_initializer=tf.zeros_initializer(),
    moving_variance_initializer=tf.ones_initializer(), beta_regularizer=None,
    gamma_regularizer=None, beta_constraint=None, gamma_constraint=None,
    renorm=False, renorm_clipping=None, renorm_momentum=0.99, fused=None,
    trainable=True, virtual_batch_size=None, adjustment=None, name=None, **kwargs
)
```

> **fused** if None or True, use a faster, fused implementation if possible. If False, use the system recommended implementation.

### tf.math.reduce_max
> **tf.math.reduce_max**\
> Computes the maximum of elements across dimensions of a tensor

```
tf.math.reduce_max(
    input_tensor, axis=None, keepdims=False, name=None
)
```

> Reduces input_tensor along the dimensions given in axis.Unless keepdims is true, the rank of the tensor is reduced by 1 for each of the entires in axis, which must be unique. If keepdims is true, the reduced dimensions are retained with length 1.

> If axis is None,all dimensions are reduced, and a tensor with a single element is returned.

> **input_tensor** The tensor to reduce.Should have real numeric type

> **axis** The dimensions to reduce.If None (the default),reduces all dimensions.Must be in range [-rank(input_tensor),rank(input_tensor)]

> **keepdims** If true,retains reduced dimensions with length 1.





> **name** A name for the operation(optional).

### tf.tile
> **tf.tile**\
> Constructs构造 a tensor by tiling平铺 a given tensor

```
tf.tile(
    input, multiples, name=None
)
```

> This operation creates a new tensor by replicating复制 `input` `multiples` times.The output tensor's i'th dimension has `input.dims(i)*multiples[i]` elements, and the values of `input` are replicated `multiples[i]` times along the i'th dimension.

> **input** A tensor,1-D or higher

> **multiples** A tensor. Must be one of the following types: int32, int64, 1-D Length must be the same as the number of dimensions in input.

> **name** A name for the operation(optional).

### tf.concat
> **tf.concat**\
> Concatenates tensors along one dimension.

```
tf.concat(
    values, axis, name='concat'
)
```
### tf.cast
> **tf.cast**\
> Cast a tensor to a new type.

```
tf.cast(
    x, dtype, name=None
)
```

> The operation supports data types (for x and dtype) of uint8, uint16, uint32, uint64, int8, int16, int32, int64, float16, float32, float64, complex64, complex128, bfloat16. In case of casting from complex types (complex64, complex128) to real types, only the real part of x is returned. In case of casting from real types to complex types (complex64, complex128), the imaginary part of the returned value is set to 0. The handling of complex types here matches the behavior of numpy.

### tf.where
> Return the elements where condition is True (multiplexing x and y).
```
tf.where(
    condition, x=None, y=None, name=None
)
```

![1](images/Screenshot%202021-01-22%2011:56:36.png)\

