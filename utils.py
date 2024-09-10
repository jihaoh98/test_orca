from math import fmod

def trueMod(a, N):
    res = fmod(a, N)
    if res < 0:
        res += N

    return res