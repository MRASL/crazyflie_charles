import thread
from multiprocessing import Pool, Manager

class Agent:
    def __init__(self, name, num):
        self.name = name
        self.id = num


def run_agent(agent):
    d = {}
    d[agent.id] = agent.name
    return agent.id, d

if __name__ == "__main__":
    agent_list = [Agent("A1", 0), Agent("A2", 1)]

    my_dict = {}


    for i in range(2):
        try:
            pool = Pool()
            res = pool.map(run_agent, agent_list)

        finally:
            pool.close()
            pool.join()

        d = {}
        for each_res in res:
            d[each_res[0]] = each_res[1]

        my_dict[i] = d


    print my_dict
