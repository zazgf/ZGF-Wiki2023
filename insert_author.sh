#!/usr/bin/env bash
#coding=utf-8

# var=$(command-name-here)
# var=`command-name-here`

# FILES=$(find . -name '*.c' -or -name '*.cpp' -or -name '*.h')
FILES=`find . -name '*.c'  -or -name '*.cu' -or -name '*.cpp' -or -name '*.h'`

for FILE in ${FILES}
do
    echo ${FILE}
    ex ${FILE} <<eof
1 insert
"""
 * @author zhangguangfeng
 * @email zgf123458@163.com
 * @create date 2020-12-29 08:56:05
 * @modify date 2020-12-29 08:56:05
 * @desc [description]
"""

.
xit
eof

done


