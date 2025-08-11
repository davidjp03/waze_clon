import queue

class Tree:
    def __init__(self, root, operators):
        self.root = root
        self.operators = operators

    def printPath(self, n):
        stack = n.pathObjective()
        path = stack.copy()
        while len(stack) != 0:
            node = stack.pop()
            if node.operator is not None:
                print(f'operador: {self.operators[node.operator]} \t estado: {node.state}')
            else:
                print(f'{node.state}')
        return path

    def reinitRoot(self):
        self.root.operator = None
        self.root.parent = None
        self.root.children = []
        self.root.level = 0

    def Aasterisk(self, endState):
        self.reinitRoot()
        pq = queue.PriorityQueue()
        pq.put((self.root.f(), self.root))
        while not pq.empty():
            node = pq.get()[1]
            children = node.getchildrens()
            for i, child in enumerate(children):
                if child is not None:
                    newChild = node.add_child(
                        value=node.value + '-' + str(i),
                        state=child,
                        operator=i
                    )
                    pq.put((newChild.f(), newChild))
                    # ✅ Comparar con el estado real del nuevo nodo
                    if newChild.state == endState:
                        return newChild
        return None  # 🔹 Si no encuentra solución
