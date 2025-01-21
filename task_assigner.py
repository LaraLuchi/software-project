class TaskAssigner:
    @staticmethod
    def greedy_assignment(robots, targets):
        """Distribuição gulosa: cada robô é atribuído ao alvo mais próximo."""
        print("Starting greedy assignment...")
        print(f"Robots: {robots}")
        print(f"Targets: {targets}")

        assignments = {}
        unassigned_targets = set(range(len(targets)))
        for i, robot in enumerate(robots):
            if not unassigned_targets:
                print("No targets left to assign.")
                break

            # Encontrar o alvo mais próximo
            closest_target = min(
                unassigned_targets,
                key=lambda t: ((robot[0] - targets[t][0]) ** 2 + (robot[1] - targets[t][1]) ** 2)
            )
            assignments[i] = closest_target
            unassigned_targets.remove(closest_target)
            print(f"Robot {i} assigned to target {closest_target}")

        print(f"Final assignments: {assignments}")
        return assignments
