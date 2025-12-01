# C++ example

In this directory, you will find a minimal example of C++ project linking against PlaCo.

## Preparing

In this example, `placo` is handled as a subdirectory, clone it here:

```
git clone https://github.com/rhoban/placo.git
```

Note that you should install PlaCo [build dependencies](https://placo.readthedocs.io/en/latest/basics/installation_source.html)

## Building

Make a `build` directory and run `cmake`:

```
mkdir build
cd build
cmake ..
make -j8
```

## Running

You can run the example using the included Python binding

```
MODULE_DIR=`echo build/lib/python3.*/site-packages/
export PYTHONPATH="$PYTHONPATH:$MODULE_DIR"
python run.py
```

And monitor the result on [http://localhost:7000/static/](http://localhost:7000/static/)