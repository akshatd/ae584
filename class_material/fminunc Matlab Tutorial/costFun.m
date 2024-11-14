function F = costFun(x, x0, y0, mu)
    F = (mu.*((x - x0).^2) + y0);
    F = F.^2;
end