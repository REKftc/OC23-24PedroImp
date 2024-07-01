package overcharged.components;

public enum propLocation {
    Middle(2), //Red
    Right(1), //Green
    Left(0); //Blue

    public int index;
    propLocation(final int index) { this.index = index; }
}