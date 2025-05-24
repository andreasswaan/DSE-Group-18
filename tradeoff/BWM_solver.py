import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

def solve_bwm_scipy_safe(criteria, best_index, worst_index, BO_vector, OW_vector):
    n = len(criteria)
    epsilon = 1e-6  # to prevent divide-by-zero

    x0 = np.ones(n) / n
    bounds = [(epsilon, 1.0) for _ in range(n)]  # avoid zero weights

    # sum of weights must be 1
    cons = {'type': 'eq', 'fun': lambda w: np.sum(w) - 1}

    def objective(w):
        errors = []
        for j in range(n):
            if j != best_index:
                bo_est = w[best_index] / (w[j] + epsilon)
                errors.append((bo_est - BO_vector[j])**2)
            if j != worst_index:
                ow_est = w[j] / (w[worst_index] + epsilon)
                errors.append((ow_est - OW_vector[j])**2)
        return np.max(errors)  # minimax approximation

    res = minimize(
        objective,
        x0,
        bounds=bounds,
        constraints=[cons],
        method='SLSQP',
        options={'maxiter': 2000, 'ftol': 1e-6}
    )

    if not res.success:
        raise ValueError(f"Optimization failed: {res.message}")

    weights = res.x
    consistency = res.fun

    return {criteria[i]: weights[i] for i in range(n)}, consistency

# === Example Usage ===

criteria = ["Safety", "Site and Setup cost", "Sustainability", "Payload handling", "Operational cost"]

# Simulate 10 people's preferences
preferences = [
    {"Name": "Felice", "best_index": 0, "worst_index": 1, "BO_vector": [1, 4, 2, 2, 1], "OW_vector": [3, 1, 3, 4, 5]},
    {"Name": "Andreas", "best_index": 0, "worst_index": 2, "BO_vector": [1, 4, 8, 2, 3], "OW_vector": [9, 3, 1, 7, 4]},
    {"Name": "Jan", "best_index": 0, "worst_index": 2, "BO_vector": [1, 5, 6, 2, 3], "OW_vector": [6, 3, 1, 5, 4]},
    {"Name": "Unknown", "best_index": 0, "worst_index": 2, "BO_vector": [1, 5, 7, 3, 3], "OW_vector": [8, 4, 1, 5, 6]},
    {"Name": "Unknown", "best_index": 0, "worst_index": 2, "BO_vector": [1, 4, 7, 2, 2], "OW_vector": [7, 3, 1, 5, 5]},
    {"Name": "Unknown", "best_index": 0, "worst_index": 2, "BO_vector": [1, 5, 8, 3, 6], "OW_vector": [8, 6, 1, 8, 7]},
    {"Name": "Unknown", "best_index": 0, "worst_index": 2, "BO_vector": [1, 5, 8, 3, 4], "OW_vector": [8, 5, 1, 7, 6]},
    {"Name": "Unknown", "best_index": 0, "worst_index": 2, "BO_vector": [1, 5, 7, 3, 3], "OW_vector": [7, 3, 1, 5, 5]},
    {"Name": "Unknown", "best_index": 0, "worst_index": 2, "BO_vector": [1, 4, 6, 2, 3], "OW_vector": [6, 2, 1, 4, 5]},
    # {"Name": "Felice", "best_index": 0, "worst_index": 2, "BO_vector": [1, 2, 4, 1, 3], "OW_vector": [4, 1, 1, 3, 2]},
]

# Store results
all_weights = []
consistencies = []

for pref in preferences:
    weights, consistency = solve_bwm_scipy_safe(
        criteria,
        pref["best_index"],
        pref["worst_index"],
        pref["BO_vector"],
        pref["OW_vector"]
    )
    all_weights.append(weights)
    consistencies.append(consistency)

# Calculate average weights
average_weights = {crit: np.mean([w[crit] for w in all_weights]) for crit in criteria}

# Plot variation in weights
plt.figure(figsize=(10, 6))
for crit in criteria:
    values = [w[crit] for w in all_weights]
    plt.plot(range(1, len(preferences) + 1), values, marker='o', label=crit)
plt.xlabel("Person Index")
plt.ylabel("Weight")
plt.title("Variation in Weights Across Preferences")
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# Plot averaged weights as a pie chart
plt.figure(figsize=(8, 8))
plt.pie(average_weights.values(), labels=average_weights.keys(), autopct='%1.1f%%', startangle=90, colors=plt.cm.Paired.colors)
plt.title("Averaged BWM Weights Distribution", fontsize=16)
plt.tight_layout()
plt.show()
