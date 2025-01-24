def closest_task_assignment(robot_positions, target_positions):
    """
    Distribui os alvos entre os robôs, garantindo que cada robô vá ao alvo mais próximo disponível,
    mesmo quando múltiplos robôs estão próximos do mesmo alvo.
    :param robot_positions: Lista de posições dos robôs.
    :param target_positions: Lista de posições dos alvos.
    :return: Um dicionário {robô_id: alvo_id}.
    """
    assignments = {}  #amazena as atribuições {robô_id: alvo_id}
    assigned_targets = set()  #alvos já atribuídos

    for robot_id, robot_pos in enumerate(robot_positions):
        closest_target = None
        min_distance = float('inf')

        #encontra o alvo mais próximo que ainda não foi atribuído
        for target_id, target_pos in enumerate(target_positions):
            if target_id not in assigned_targets:  # Verifica se o alvo já foi atribuído
                distance = robot_pos.dist_to(target_pos)
                if distance < min_distance:
                    closest_target = target_id
                    min_distance = distance

        #atribui o alvo mais próximo disponível ao robô
        if closest_target is not None:
            assignments[robot_id] = closest_target
            assigned_targets.add(closest_target)  #marca o alvo como atribuído

    return assignments
