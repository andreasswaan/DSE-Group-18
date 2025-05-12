import os
import numpy as np
import matplotlib.pyplot as plt
from globals import main_dir

plot_path =  os.path.join(main_dir, "tradeoff")
os.makedirs(plot_path, exist_ok=True)
# Define the criteria and concepts
criteria = ["Production cost", "Operational cost", "Safety", "Sustainability", "Payload handling"]
concepts = ["Multicopter", "Fixed-wing slingshot", "Fixed wing with hor. & ver. propellers", "Tilt-rotor"]
 
# Scores for each concept on each criterion (rows: concepts, columns: criteria)
scores = np.array([
    [4, 3, 4, 2, 5],  # Concept A
    [2, 4, 4, 3, 1],  # Concept B
    [4, 4, 3, 4, 4],  # Concept C
    [3, 3, 2, 4, 4]   # Concept D
])

# Initial weights for the criteria (must sum to 1)
weights = np.array([0.1, 0.25, 0.3, 0.1, 0.25])

# Function to calculate the total score for each concept
def calculate_scores(scores, weights):
    return np.dot(scores, weights)

# Sensitivity study
num_simulations = 1000
winners = []

for _ in range(num_simulations):
    # Perturb the weights slightly
    perturbation = np.random.normal(0, 0.08, len(weights))  # Small random changes
    new_weights = weights + perturbation
    new_weights = np.abs(new_weights)  # Ensure no negative weights
    new_weights /= np.sum(new_weights)  # Normalize to sum to 1

    # Calculate scores with the new weights
    total_scores = calculate_scores(scores, new_weights)
    winner = np.argmax(total_scores)  # Index of the winning concept
    winners.append(winner)

# Count the frequency of each concept being the winner
winner_counts = np.bincount(winners, minlength=len(concepts))

# Plot the results with enhanced colors
plt.figure(figsize=(8, 6))

# Use a colormap to assign distinct colors to each bar
colors = plt.cm.viridis(np.linspace(0, 1, len(concepts)))

plt.bar(concepts, winner_counts, color=colors, edgecolor='black')
plt.xlabel("Concepts")
plt.ylabel("Frequency of Winning")
plt.title("Sensitivity Study: Frequency of Winning Concepts")
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.tight_layout()
plt.savefig(os.path.join(plot_path, "sensitivity_gaussian.png"), dpi=300, bbox_inches='tight')


# Calculate standard results (scores with original weights)
standard_scores = calculate_scores(scores, weights)

# Initialize a matrix to store scores for each design after perturbing each criterion
perturbed_scores = []

for i, crit in enumerate(criteria):
    # Add 20% error to the current criterion
    perturbed_weights = weights.copy()
    perturbed_weights[i] += 0.2 * weights[i]
    perturbed_weights = np.abs(perturbed_weights)  # Ensure no negative weights
    perturbed_weights /= np.sum(perturbed_weights)  # Normalize to sum to 1

    # Calculate scores with the perturbed weights
    total_scores = calculate_scores(scores, perturbed_weights)
    perturbed_scores.append(total_scores)

# Convert to a NumPy array for easier manipulation
perturbed_scores = np.array(perturbed_scores)  # Shape: (num_criteria, num_concepts)

# Plot the results
x = np.arange(len(criteria) + 1)  # Add one position for the standard results
width = 0.2  # Width of each bar (adjusted for multiple designs)

plt.figure(figsize=(14, 8))

# Use a colormap to assign distinct colors to each design
colors = plt.cm.Set2(np.linspace(0, 1, len(concepts)))  # Use a new colormap (Set2)

# Plot standard results as the first group
for j, concept in enumerate(concepts):
    plt.bar(
        x[0] + j * width - (width * (len(concepts) - 1) / 2),  # Center the bars for standard results
        [standard_scores[j]],  # Standard scores
        width,
        label=f"{concept} (Standard)",
        color=colors[j],
        alpha=0.7  # Slight transparency for standard bars
    )

# Plot perturbed results for each criterion
for j, concept in enumerate(concepts):
    plt.bar(
        x[1:] + j * width - (width * (len(concepts) - 1) / 2),  # Offset for perturbed bars
        perturbed_scores[:, j],
        width,
        label=f"{concept} (Perturbed)" if j == 0 else None,  # Add label only once
        color=colors[j]
    )

# Update x-axis labels
plt.xlabel("Criteria")
plt.ylabel("Scores")
plt.title("Scores per Design (Standard vs. 20% Sensitivity)")
plt.xticks(x, ["Standard"] + criteria)  # Add "Standard" as the first label
plt.legend(title="Designs")
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.tight_layout()
plt.savefig(os.path.join(plot_path, "sensitivity_linear.png"), dpi=300, bbox_inches='tight')
