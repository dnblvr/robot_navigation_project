#include <stdio.h>
#include <stdlib.h>

// Define the structure for a decision tree node
typedef struct Node {
  int     feature_index;  // Index of the feature to split on
  float   threshold;      // Threshold value for the split
  int     class;          // Class label (for leaf nodes)
  struct  Node* left;     // Pointer to the left child (values <= threshold)
  struct  Node* right;    // Pointer to the right child (values > threshold)
  uint8_t is_leaf;        // Flag to indicate if the node is a leaf
} Node;

// Function to create a new internal node
Node* create_internal_node(int feature_index, float threshold) {
    Node* new_node = (Node*)malloc(sizeof(Node));
    if (!new_node) {
        perror("Failed to allocate memory for node");
        exit(EXIT_FAILURE);
    }
    new_node->feature_index = feature_index;
    new_node->threshold = threshold;
    new_node->value = 0.0;  // Not used for internal nodes
    new_node->left = NULL;
    new_node->right = NULL;
    new_node->is_leaf = 0;  // Not a leaf
    return new_node;
}

// Function to create a new leaf node
Node* create_leaf_node(float value) {
    Node* new_node = (Node*)malloc(sizeof(Node));
    if (!new_node) {
        perror("Failed to allocate memory for node");
        exit(EXIT_FAILURE);
    }
    new_node->feature_index = -1;  // Not used for leaf nodes
    new_node->threshold = 0.0;     // Not used for leaf nodes
    new_node->value = value;       // Prediction value
    new_node->left = NULL;
    new_node->right = NULL;
    new_node->is_leaf = 1;         // Is a leaf
    return new_node;
}

// Function to free the memory of a node and its children
void free_tree(Node* root) {
    if (root) {
        free_tree(root->left);
        free_tree(root->right);
        free(root);
    }
}

// Function to traverse the tree and make a prediction
float predict(Node* root, float* features) {
    if (!root) {
        fprintf(stderr, "Error: Tree is empty.\n");
        exit(EXIT_FAILURE);
    }

    if (root->is_leaf) {
        return root->value;  // Return the prediction value for leaf nodes
    }

    if (features[root->feature_index] <= root->threshold) {
        return predict(root->left, features);  // Traverse left
    } else {
        return predict(root->right, features); // Traverse right
    }
}

// Example usage
int main() {
    // Create a simple decision tree
    Node* root = create_internal_node(0, 5.0);  // Split on feature 0 with threshold 5.0
    root->left = create_leaf_node(10.0);        // Prediction value for left leaf
    root->right = create_leaf_node(20.0);       // Prediction value for right leaf

    // Example feature vector
    float features[1] = {4.0};  // Feature 0 = 4.0

    // Make a prediction
    float prediction = predict(root, features);
    printf("Prediction: %.2f\n", prediction);

    // Free memory
    free_tree(root);

    return 0;
}