import warnings

# NumPy on Apple's Accelerate BLAS raises floating-point flags on finite data in matmul ("divide by
# zero / overflow / invalid value encountered in matmul", arrays of ~1000+ rows, results finite):
# false alarms. Only matmul's are silenced; the same warnings from anything else still show.
warnings.filterwarnings("ignore", message=r".* encountered in matmul", category=RuntimeWarning)
