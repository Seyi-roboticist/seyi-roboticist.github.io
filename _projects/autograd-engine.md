---
layout: project
title: "Autograd Engine & Neural Network Classifier"
description: "A from-scratch implementation of automatic differentiation and backpropagation — the core engine behind modern deep learning. Includes a scalar-valued autograd engine, linear layer, softmax cross-entropy loss, and a gradient descent training loop that achieves 99% accuracy on a 4-class 2D classification task."
date: 2024-10-15
categories: [Machine Learning, Deep Learning, Python]
featured_image: "/assets/images/projects/autograd-engine/comp_graph.png"
github_url: "https://github.com/Seyi-roboticist"

gallery:
  - type: "image"
    file: "/assets/images/projects/autograd-engine/comp_graph.png"
    description: "Full computational graph showing forward values and backward gradients for f = σ(w₁x₁ + w₂x₂) + 0.5(w₁² + w₂²)"

code_files:
  - name: "Value (Autograd Core)"
    file: "value.py"
    language: "python"
    content: |
      class Value:
          """Basic unit storing a single scalar value and its gradient."""

          def __init__(self, data, _children=()):
              self.data = data
              self.grad = 0
              self._prev = set(_children)
              self._backward = lambda: None

          def __add__(self, other):
              other = other if isinstance(other, Value) else Value(other)
              out = Value(self.data + other.data, (self, other))
              def _backward():
                  self.grad += out.grad * 1.0
                  other.grad += out.grad * 1.0
              out._backward = _backward
              return out

          def __mul__(self, other):
              other = other if isinstance(other, Value) else Value(other)
              out = Value(self.data * other.data, (self, other))
              def _backward():
                  self.grad += other.data * out.grad
                  other.grad += self.data * out.grad
              out._backward = _backward
              return out

          def __pow__(self, n):
              assert isinstance(n, (int, float))
              out = Value(self.data ** n, (self,))
              def _backward():
                  self.grad += (n * self.data ** (n - 1)) * out.grad
              out._backward = _backward
              return out

          def exp(self):
              out = Value(np.exp(self.data), (self,))
              def _backward():
                  self.grad += out.data * out.grad
              out._backward = _backward
              return out

          def log(self):
              out = Value(np.log(self.data), (self,))
              def _backward():
                  self.grad += (1 / self.data) * out.grad
              out._backward = _backward
              return out

          def relu(self):
              out = Value(0 if self.data < 0 else self.data, (self,))
              def _backward():
                  self.grad += (out.data > 0) * out.grad
              out._backward = _backward
              return out

          def backward(self):
              self.grad = 1
              topo, visited = [], set()
              def build_topo(v):
                  if v not in visited:
                      visited.add(v)
                      for child in v._prev:
                          build_topo(child)
                      topo.append(v)
              build_topo(self)
              for v in reversed(topo):
                  v._backward()

          def __neg__(self):
              return self * -1
          def __radd__(self, other):
              return self + other
          def __sub__(self, other):
              return self + (-other)
          def __rsub__(self, other):
              return other + (-self)
          def __rmul__(self, other):
              return self * other
          def __truediv__(self, other):
              return self * other ** -1
          def __rtruediv__(self, other):
              return other * self ** -1
          def __repr__(self):
              return f"Value(data={self.data}, grad={self.grad})"

  - name: "Linear Layer & Training"
    file: "model.py"
    language: "python"
    content: |
      class Module:
          def parameters(self):
              return []
          def zero_grad(self):
              for p in self.parameters():
                  p.grad = 0

      class LinearLayer(Module):
          def __init__(self, nin, nout):
              self.w = [[Value(random.uniform(-1, 1))
                          for j in range(nout)]
                         for i in range(nin)]
              self.b = [Value(0) for _ in range(nout)]

          def __call__(self, x):
              x_out = []
              for i in range(len(x)):
                  row = []
                  for j in range(len(self.b)):
                      val = sum(x[i][k] * self.w[k][j]
                                for k in range(len(self.w)))
                      row.append(val + self.b[j])
                  x_out.append(row)
              return x_out

          def parameters(self):
              return [p for row in self.w for p in row] + self.b

      def softmax(y_hat):
          s = []
          for row in y_hat:
              exp_row = [Value.exp(x) for x in row]
              exp_sum = sum(exp_row)
              s.append([e / exp_sum for e in exp_row])
          return s

      def cross_entropy_loss(y_hat, y):
          y_prob = softmax(y_hat)
          eps = 1e-6
          losses = [-Value.log(y_prob[i][y[i]] + eps)
                     for i in range(len(y))]
          return sum(losses) * (1 / len(y))

      def train(x, y, model, lr=1, epochs=500):
          for i in range(epochs):
              y_hat = model(x)
              loss = cross_entropy_loss(y_hat, y)
              model.zero_grad()
              loss.backward()
              for p in model.parameters():
                  p.data -= lr * p.grad
---

## Project Overview

This project implements a **scalar-valued automatic differentiation (autograd) engine** entirely from scratch in Python, with no framework dependencies beyond NumPy. The engine records arithmetic operations as they occur, constructs a dynamic computational graph, and propagates gradients backward through the graph using the chain rule — the same mechanism that powers PyTorch, TensorFlow, and JAX under the hood.

The autograd engine was then used to build and train a **linear classifier** on a 2D, 4-class dataset, achieving **99% accuracy** within 500 gradient descent iterations.

## How It Works

### The `Value` Class

Every scalar in the computation is wrapped in a `Value` object. Each operation — addition, multiplication, exponentiation, `exp`, `log`, `relu` — creates a new `Value`, links it to its parent nodes via `_prev`, and stores a `_backward` closure that encodes the local gradient rule. Calling `.backward()` on the final output triggers a topological sort of the graph and executes each `_backward` in reverse order, accumulating gradients all the way back to the leaf variables.

### Supported Operations and Their Local Gradients

| Operation | Forward | Local Gradient (∂out/∂input) |
|-----------|---------|------------------------------|
| `a + b`   | `a.data + b.data` | `1.0` for both `a` and `b` |
| `a * b`   | `a.data * b.data` | `b.data` for `a`, `a.data` for `b` |
| `a ** n`  | `a.data ** n` | `n * a.data^(n-1)` |
| `exp(a)`  | `e^(a.data)` | `e^(a.data)` (i.e., `out.data`) |
| `log(a)`  | `ln(a.data)` | `1 / a.data` |
| `relu(a)` | `max(0, a.data)` | `1` if `a.data > 0`, else `0` |

### Backpropagation via Topological Sort

The `backward()` method performs a depth-first traversal to produce a topological ordering of all nodes in the computational graph. It then iterates in reverse order, applying the chain rule at each node: the gradient of a node's input is accumulated as the product of the upstream gradient and the local gradient.

### Computational Graph Example

The function `f = σ(w₁x₁ + w₂x₂) + 0.5(w₁² + w₂²)` combines a sigmoid activation with L2 weight regularization. With inputs `w₁ = 0.2`, `w₂ = 0.4`, `x₁ = -0.4`, `x₂ = 0.5`:

| Variable | Expression | Forward Value | Gradient (∂f/∂·) |
|----------|------------|:-------------:|:-----------------:|
| w₁       | input      | 0.2000 | 0.1004 |
| w₂       | input      | 0.4000 | 0.5246 |
| x₁       | input      | −0.4000 | 0.0498 |
| x₂       | input      | 0.5000 | 0.0996 |
| z        | w₁x₁ + w₂x₂ | 0.1200 | 0.2491 |
| R        | 0.5(w₁² + w₂²) | 0.1000 | 1.0000 |
| G = σ(z) | 1/(1+e⁻ᶻ) | 0.5300 | 1.0000 |
| **f**    | **G + R**  | **0.6300** | **1.0000** |

## Neural Network Training

### Architecture

A single `LinearLayer(2, 4)` maps 2D input features to 4 output logits. The logits are passed through `softmax` to produce class probabilities, and the `cross_entropy_loss` computes the negative log-likelihood against ground truth labels.

### Training Loop

Each iteration consists of: forward pass → loss computation → `zero_grad()` → `backward()` → parameter update (`p.data -= lr * p.grad`). With a learning rate of 1.0, the model converges from random initialization to 99% accuracy on 100 training samples across 4 classes.

### Results

| Metric | Value |
|--------|:-----:|
| Final Accuracy | **99%** |
| Final Loss | **0.13** |
| Training Iterations | 500 |
| Number of Classes | 4 |
| Dataset Size | 100 samples |
| Input Dimensions | 2D |

## Technical Details

**Language:** Python 3 with NumPy

**Key Concepts Demonstrated:**
- Automatic differentiation (reverse mode)
- Computational graph construction and traversal
- Topological sorting (DFS-based)
- Chain rule for gradient accumulation
- Softmax and cross-entropy loss derivation
- Stochastic gradient descent optimization

**No external ML frameworks** were used — every gradient is analytically derived and implemented by hand.
