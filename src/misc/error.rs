#[derive(Debug, Clone)]
pub enum Report<T, E> {
    Ok(T),
    Warn { value: T, error: E },
}

impl<T, E> Report<T, E> {
    pub fn new(value: T) -> Self { Self::Ok(value) }

    pub fn from_parts(value: T, error: E) -> Self { Self::Warn { value, error } }

    pub fn has_errors(&self) -> bool { matches!(self, Self::Warn { .. }) }

    pub fn is_ok(&self) -> bool { matches!(self, Self::Ok(_)) }

    pub fn map<U>(self, f: impl FnOnce(T) -> U) -> Report<U, E> {
        match self {
            Report::Ok(v) => Report::Ok(f(v)),
            Report::Warn { value, error } => Report::Warn {
                value: f(value),
                error,
            },
        }
    }

    pub fn into_parts(self) -> (T, Option<E>) {
        match self {
            Report::Ok(v) => (v, None),
            Report::Warn { value, error } => (value, Some(error)),
        }
    }

    pub fn value(self) -> T {
        match self {
            Report::Ok(v) => v,
            Report::Warn { value, .. } => value,
        }
    }
}
