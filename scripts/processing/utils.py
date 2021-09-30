def topic_to_array(bag, topic):
    array = []
    for topic, msg, t in bag.read_messages([topic]):
        array.append(msg)
    return array
