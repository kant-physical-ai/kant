import numpy as np

def main():
    a = np.array([[1,2,3], [4,5,6]])
    print(a.T) #전치행렬
    print(a.T.shape)
    print(-a)
    print(-a.T)
if __name__ == '__main__':
    main()