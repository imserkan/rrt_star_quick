class Point(object):

    def __init__(self, coord, cost, parent):
        self.coord = coord
        self.cost = cost
        self.parent = parent

    def setCoord(self, coord):
        self.coord = coord

    def setCost(self, cost):
        self.cost = cost

    def setParent(self, parent):
        self.parent = parent

    def getCoord(self):
        return self.coord

    def getCost(self):
        return self.cost

    def getParent(self):
        return self.parent