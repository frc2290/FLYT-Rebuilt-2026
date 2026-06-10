# Future Idea: Learned Latent Spaces for Comparison Selection

This idea explores adding learned latent spaces to help select comparisons in the app. The key design principle is to keep sampling logic grounded in standard vector math: `sqlite_vec` and the Streamlit samplers only need arrays of floats and standard distance metrics such as cosine similarity or L2 distance. If the vector source is abstracted cleanly, the app can hot-swap the underlying geometry without changing frontend comparison-selection logic.

By abstracting the vector source, the app becomes a pure active-learning platform. Samplers continue to ask for nearby vectors, while the selected vector space determines what “nearby” means.

## The Three Similarity Spaces

The math can remain identical across all three spaces, but the meaning of the distance changes depending on which vector is fed into the sampler.

### 1. OpenAI Embedding: The Global Prior

- **What it is:** A raw OpenAI embedding vector, such as a 1536-dimensional vector representing the internet's broad consensus about a concept.
- **Sampler behavior:** When samplers such as “Balanced Surprise” or “Dense Triad” operate in this space, they find items that sound similar in generic plain text. This space is blind to app-specific household rules, relational labels, and hierarchy goals.
- **Best for:** Bootstrapping a new domain or finding items that are conceptually adjacent but have not yet been sorted relationally.

### 2. Shared Encoder: The Fine-Tuned Semantic Core

- **What it is:** The OpenAI vector after passing through the model's initial dense layers, before it is routed to domain-specific Poincare projections.
- **Sampler behavior:** This space represents the model's learned semantic grouping from triplets. Running a Dense Triad sampler here finds items that the model currently believes are functionally identical.
- **Best for:** Ironing out semantic confusion. For example, if the model thinks a “Rubber Mallet” and a “Claw Hammer” are effectively the same in the Shared Encoder, presenting them in a triad gives the user an opportunity to provide the negative push that separates them in a later training epoch.

### 3. Log-Mapped Poincare: The Relational Tangent Space

- **What it is:** The model's final, domain-specific hyperbolic coordinates, where radius can encode hierarchy, flattened into a Euclidean tangent space.
- **The math:** Standard cosine similarity should not be run directly on raw Poincare vectors because the space curves exponentially near the edges. Instead, project vectors onto the tangent space at the origin using the logarithmic map:

  ```text
  v = (arctanh(||x||) / ||x||) * x
  ```

  The result is a standard Euclidean vector `v` that can be used by existing vector-distance queries.
- **Sampler behavior:** This is the strongest active-learning space. Distance here accounts for both similarity and hierarchy. If two vectors are close in this tangent space, the model has placed them in the same relational bucket at the same depth of specificity.
- **Best for:** Refining final edge boundaries. Sampling in this space directly targets the model loss function's current weakest points.

## Architectural Implementation

Because all three approaches resolve to standard Euclidean vector arrays, the implementation can stay clean and minimally invasive.

### 1. Database Schema

`sqlite_vec` virtual tables require a fixed dimension size. Since OpenAI embeddings and learned latent vectors may use different dimensions, store each space in a separate virtual table:

- `vec_openai`, for example `dim=1536`
- `vec_shared_encoder`, for example `dim=32`
- `vec_poincare_tangent`, for example `dim=32`

### 2. Vector Adapter

In the Streamlit backend, such as `data_access.py` or a similar data-access layer, update similarity queries to accept a `vector_space` argument.

When the UI requests the top nearest neighbors for a sampler such as Dense Triad, the adapter routes the SQL query to the matching virtual table. The distance operation, such as `vec_distance_cosine`, can remain unchanged.

### 3. Inference Pipeline

Add a script that runs alongside standard CSV artifact generation after each training epoch. The script should:

1. Load the latest `.pt` checkpoint.
2. Pass all entities through the model.
3. Extract Shared Encoder outputs.
4. Extract final Poincare outputs.
5. Apply the logarithmic map to create Poincare tangent-space vectors.
6. Bulk-upsert the generated vectors into the SQLite database.

With this setup, UI samplers remain simple: they only ask for close vectors. The intelligence of what “close” means is decoupled from the frontend and driven by whichever model-derived vector table is selected in the backend.
