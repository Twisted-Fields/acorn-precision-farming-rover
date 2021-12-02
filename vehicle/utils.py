def AppendFIFO(list, value, max_values):
    list.append(value)
    while len(list) > max_values:
        list.pop(0)
    return list

def clamp(v, low, high):
    """clamp v to the range of [low, high]"""
    return max(low, min(v, high))
