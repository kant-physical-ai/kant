from libs import math_libs, numpy_libs, device_libs, system_libs


def main():
    print("hello world")
    print(f"submodules: {numpy_libs.__name__}, {system_libs.__name__}")


if __name__ == "__main__":
    main()