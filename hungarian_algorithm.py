import numpy as np

class Hungarian:
    @staticmethod
    def solve(cost_matrix):
        """
        Resolve o problema de atribuição usando o algoritmo Húngaro.
        Args:
            cost_matrix (np.ndarray): Matriz de custo.
        Returns:
            list[tuple[int, int]]: Lista de pares (índice do robô, índice do alvo).
        """
        n, m = cost_matrix.shape
        row_mask = np.zeros(n, dtype=bool)
        col_mask = np.zeros(m, dtype=bool)
        row_assign = -np.ones(n, dtype=int)
        col_assign = -np.ones(m, dtype=int)

        #subtração de mínimos das linhas
        for i in range(n):
            cost_matrix[i] -= cost_matrix[i].min()

        #subtração de mínimos das colunas
        for j in range(m):
            cost_matrix[:, j] -= cost_matrix[:, j].min()

        #cobertura inicial de zeros com linhas e colunas
        while True:
            zero_count = (cost_matrix == 0).sum(axis=1)
            for i in np.argsort(zero_count):
                for j in range(m):
                    if cost_matrix[i, j] == 0 and not row_mask[i] and not col_mask[j]:
                        row_assign[i] = j
                        col_assign[j] = i
                        row_mask[i] = True
                        col_mask[j] = True
                        break

            #verifica se todos os robôs foram atribuídos
            if np.all(row_mask) or np.all(col_mask):
                break

            #encontra as linhas e colunas descobertas
            uncovered_rows = np.where(~row_mask)[0]
            uncovered_cols = np.where(~col_mask)[0]

            #se não houver linhas ou colunas descobertas, interrompe
            if uncovered_rows.size == 0 or uncovered_cols.size == 0:
                break

            #calcula o mínimo elemento não coberto
            min_uncovered = cost_matrix[np.ix_(uncovered_rows, uncovered_cols)].min()
            if min_uncovered == 0 or np.isnan(min_uncovered):
                break

            #ajusta a matriz para cobrir mais zeros
            for i in uncovered_rows:
                cost_matrix[i] -= min_uncovered
            for j in uncovered_cols:
                cost_matrix[:, j] += min_uncovered

        #gera a lista de pares de atribuição
        assignments = [(i, row_assign[i]) for i in range(n) if row_assign[i] != -1]
        return assignments
