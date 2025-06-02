import heapq

class Mapa2D:
    def __init__(self, largura, altura):
        self.largura = largura
        self.altura = altura
        self.grid = [[0 for _ in range(largura)] for _ in range(altura)]

    def marcar_obstaculo(self, x, y):
        if 0 <= x < self.largura and 0 <= y < self.altura:
            self.grid[y][x] = 1

    def esta_livre(self, x, y):
        return 0 <= x < self.largura and 0 <= y < self.altura and self.grid[y][x] == 0
    
    def imprimir(self):
        print("Mapa: ")
        for linha in self.grid:
            print(' '.join(str(c) for c in linha))

def heuristica(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_estrela(mapa, inicio, objetivo):
    fila = []
    heapq.heappush(fila, (0, inicio))
    veio_de = {inicio: None}
    custo = {inicio: 0}

    while fila:
        _, atual = heapq.heappop(fila)

        if atual == objetivo:
            break

        for dx, dy in [(0,1),(1,0),(0,-1),(-1,0)]:
            vizinho = (atual[0]+dx, atual[1]+dy)
            if mapa.esta_livre(*vizinho):
                novo_custo = custo[atual] + 1
                if vizinho not in custo or novo_custo < custo[vizinho]:
                    custo[vizinho] = novo_custo
                    prioridade = novo_custo + heuristica(vizinho, objetivo)
                    heapq.heappush(fila, (prioridade, vizinho))
                    veio_de[vizinho] = atual
    
    caminho = []
    atual = objetivo
    while atual != inicio:
        if atual not in veio_de:
            return[] # Não há caminho
        caminho.append(atual)
        atual = veio_de[atual]
    caminho.reverse()
    return caminho

# Teste
if __name__ == '__main__':
    mapa = Mapa2d(10,10)
    mapa.marcar_obstaculo(4,5)
    mapa.marcar_obstaculo(4,6)
    mapa.marcar_obstaculo(4,7)
    mapa.imprimir()
    caminho = a_estrela(mapa, (0, 0), (7, 7))

    print("Caminho:", caminho)
