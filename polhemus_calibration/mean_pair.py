s = """

# to_poses(lines)
# range
# mean
# compare

def p():
        %paste lines
        return  to_poses(lines)

"""

print s


def compare(d1, d2):
        m1 = mean(d1)
        m2 = mean(d2)
        return range([m1, m2])

def mean(data):
        z = zip(*data)
        r = [min(x) + 0.5 * (max(x) - min(x)) for x in z]
        return r

def range(data):
        z = zip(*data)
        r = [max(x) - min(x) for x in z]
        return r

def to_poses(lines):
        poses =[]
        for line in lines:
                tokens = line.split()
                print tokens
                values = [float(s) for s in tokens]
                poses.append(values[1:])
        return poses
